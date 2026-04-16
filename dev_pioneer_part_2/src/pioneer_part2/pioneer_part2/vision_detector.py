"""
Vision detector node for Pioneer P3-AT Part 2.

Capabilities
------------
1. /capture_photo  (std_srvs/Empty request, returns empty response)
   Saves the current camera frame to ``photo_save_dir/wp_<n>.jpg``.

2. /detect_object  (std_srvs/Empty request, returns empty response)
   - Detects the nearest non-orange coloured object in the camera frame
   - Classifies its shape (circle / triangle / rectangle / unknown)
   - Estimates its distance using depth image (OAK-D), LiDAR bearing match,
     or focal-length formula — tried in that priority order
   - Logs and stores the result for the journey summary

Results are published on /detection_result (std_msgs/String) as JSON-formatted text
so MissionController can consume them.

Parameters
----------
  camera_topic    str   default "camera"             — RGB topic
  depth_topic     str   default "oak/stereo/image_raw" — depth 16-UC1 or 32FC1
  use_depth       bool  default false               — enable depth-based distance
  photo_save_dir  str   default "~/mission_photos"

Subscriptions
-------------
  <camera_topic>   sensor_msgs/Image           (configurable, default /camera)
  <camera_topic>_info  sensor_msgs/CameraInfo
  /scan            sensor_msgs/LaserScan
  <depth_topic>    sensor_msgs/Image            (optional, OAK-D stereo depth)

Services
--------
  /capture_photo   std_srvs/Empty
  /detect_object   std_srvs/Empty

Published
---------
  /detection_result   std_msgs/String   (JSON: {wp, color, shape, distance_m, photo_path})
"""

import json
import math
import os
from datetime import datetime

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


# ---- colour ranges in HSV space ----
COLOR_RANGES = {
    'orange': ([5,  150, 100], [20, 255, 255]),
    'red1':   ([0,  150, 100], [5,  255, 255]),   # red wraps around 0
    'red2':   ([170, 150, 100], [180, 255, 255]),
    'yellow': ([22, 120, 100], [35, 255, 255]),
    'green':  ([40, 80,  60],  [80, 255, 255]),
    'blue':   ([100, 80, 60],  [130, 255, 255]),
    'white':  ([0,  0,  200],  [180, 30, 255]),
}

DISPLAY_COLORS = {
    'red':    'red',
    'yellow': 'yellow',
    'green':  'green',
    'blue':   'blue',
    'white':  'white',
}


def _classify_shape(contour) -> str:
    """Classify a contour into circle / triangle / rectangle / unknown."""
    peri = cv2.arcLength(contour, True)
    if peri < 1.0:
        return 'unknown'
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    n = len(approx)
    if n == 3:
        return 'triangle'
    elif n == 4:
        return 'rectangle'
    elif n >= 8:
        # circularity check
        area = cv2.contourArea(contour)
        circularity = 4 * math.pi * area / (peri * peri) if peri > 0 else 0
        if circularity > 0.7:
            return 'circle'
    return 'unknown'


def _dominant_color(hsv_roi) -> str:
    """Return the dominant non-orange colour name from an HSV ROI."""
    best_label = 'unknown'
    best_count = 0
    for label, (lo, hi) in COLOR_RANGES.items():
        if label in ('orange', 'red1', 'red2'):
            continue
        mask = cv2.inRange(hsv_roi, np.array(lo), np.array(hi))
        count = int(np.sum(mask > 0))
        if count > best_count:
            best_count = count
            best_label = label

    # Merge red1 + red2
    mask_r1 = cv2.inRange(hsv_roi,
                           np.array(COLOR_RANGES['red1'][0]),
                           np.array(COLOR_RANGES['red1'][1]))
    mask_r2 = cv2.inRange(hsv_roi,
                           np.array(COLOR_RANGES['red2'][0]),
                           np.array(COLOR_RANGES['red2'][1]))
    red_count = int(np.sum((mask_r1 | mask_r2) > 0))
    if red_count > best_count:
        best_count = red_count
        best_label = 'red'

    return best_label if best_count > 50 else 'unknown'


class VisionDetector(Node):

    def __init__(self):
        super().__init__('vision_detector')

        self.declare_parameter('photo_save_dir', os.path.expanduser('~/mission_photos'))
        self.declare_parameter('orange_h_low', 5)
        self.declare_parameter('orange_h_high', 20)
        self.declare_parameter('orange_s_low', 150)
        self.declare_parameter('orange_v_low', 100)
        self.declare_parameter('cone_real_height', 0.32)
        self.declare_parameter('min_contour_area', 200)
        # Configurable camera topic — default matches Gazebo bridge and sim launch.
        # Override to e.g. "oak/rgb/image_raw" for OAK-D without a topic remap,
        # or keep at "camera" and remap at launch level (recommended).
        self.declare_parameter('camera_topic', 'camera')
        # Optional OAK-D stereo depth stream for more accurate distance estimation.
        # Set use_depth: true in mission_params_robot.yaml to enable.
        self.declare_parameter('depth_topic', 'oak/stereo/image_raw')
        self.declare_parameter('use_depth', False)

        self._photo_dir = os.path.expanduser(
            self.get_parameter('photo_save_dir').value)
        self._cone_h = self.get_parameter('cone_real_height').value
        self._min_area = self.get_parameter('min_contour_area').value
        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        use_depth = self.get_parameter('use_depth').value

        self._bridge = CvBridge() if CvBridge is not None else None
        self._latest_image: np.ndarray | None = None
        self._latest_depth: np.ndarray | None = None   # depth float32 (metres)
        self._camera_info: CameraInfo | None = None
        self._latest_scan: LaserScan | None = None
        self._photo_count = 0

        # Stored detections for journey summary
        self._detections: list[dict] = []

        if self._bridge is None:
            self.get_logger().warn('cv_bridge not available — photo/detection disabled')

        # RGB + CameraInfo subscriptions (topic-configurable for OAK-D)
        self.create_subscription(Image, camera_topic, self._image_cb, 1)
        # CameraInfo topic: append _info if camera_topic ends with /image_raw,
        # otherwise use camera_topic + "_info" convention.
        if camera_topic.endswith('/image_raw'):
            info_topic = camera_topic[:-len('/image_raw')] + '/camera_info'
        elif camera_topic == 'camera':
            info_topic = 'camera_info'
        else:
            info_topic = camera_topic + '_info'
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 1)

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 1)

        # Optional depth subscription (OAK-D stereo)
        if use_depth:
            self.create_subscription(Image, depth_topic, self._depth_cb, 1)
            self.get_logger().info(f'Depth-based distance enabled: {depth_topic}')

        self._result_pub = self.create_publisher(String, 'detection_result', 10)

        self.create_service(Empty, 'capture_photo', self._capture_cb)
        self.create_service(Empty, 'detect_object', self._detect_cb)

        self.get_logger().info(
            f'Vision detector ready — camera: {camera_topic} — '
            f'photos saved to {self._photo_dir}')

    # ---- subscriptions -------------------------------------------------------

    def _image_cb(self, msg: Image):
        if self._bridge is None:
            return
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion error: {e}')

    def _info_cb(self, msg: CameraInfo):
        self._camera_info = msg

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _depth_cb(self, msg: Image):
        """Receive OAK-D stereo depth image (16UC1 in mm or 32FC1 in m)."""
        if self._bridge is None:
            return
        try:
            # 16UC1 (millimetres) — common OAK-D stereo encoding
            if msg.encoding in ('16UC1', '16uc1', 'mono16'):
                depth_mm = self._bridge.imgmsg_to_cv2(msg, '16UC1')
                self._latest_depth = depth_mm.astype(np.float32) / 1000.0
            elif msg.encoding in ('32FC1', '32fc1'):
                self._latest_depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                self.get_logger().warn(
                    f'Unsupported depth encoding: {msg.encoding}',
                    throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().warn(f'Depth image conversion error: {e}')

    # ---- photo capture service -----------------------------------------------

    def _capture_cb(self, _req, resp):
        if self._latest_image is None:
            self.get_logger().warn('capture_photo: no image available')
            return resp

        os.makedirs(self._photo_dir, exist_ok=True)
        self._photo_count += 1
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self._photo_dir, f'wp_{self._photo_count}_{ts}.jpg')
        cv2.imwrite(filename, self._latest_image)
        self.get_logger().info(f'Photo saved: {filename}')

        result = json.dumps({'type': 'photo', 'wp': self._photo_count,
                              'path': filename})
        self._result_pub.publish(String(data=result))
        return resp

    # ---- object detection service --------------------------------------------

    def _detect_cb(self, _req, resp):
        if self._latest_image is None:
            self.get_logger().warn('detect_object: no image available')
            result = json.dumps({'type': 'detection', 'status': 'no_image'})
            self._result_pub.publish(String(data=result))
            return resp

        detection = self._run_detection(self._latest_image)
        self._detections.append(detection)

        result_str = json.dumps(detection)
        self._result_pub.publish(String(data=result_str))
        self.get_logger().info(
            f"Detection: color={detection.get('color','?')}  "
            f"shape={detection.get('shape','?')}  "
            f"dist={detection.get('distance_m','?')} m")
        return resp

    # ---- detection logic -----------------------------------------------------

    def _run_detection(self, bgr: np.ndarray) -> dict:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Build orange mask to exclude cone from the search
        orange_lo = np.array([
            self.get_parameter('orange_h_low').value,
            self.get_parameter('orange_s_low').value,
            self.get_parameter('orange_v_low').value,
        ])
        orange_hi = np.array([
            self.get_parameter('orange_h_high').value, 255, 255])
        orange_mask = cv2.inRange(hsv, orange_lo, orange_hi)
        # Exclude orange pixels from detection image
        search = bgr.copy()
        search[orange_mask > 0] = 0

        hsv_search = cv2.cvtColor(search, cv2.COLOR_BGR2HSV)

        # Try each non-orange colour
        best_contour = None
        best_area = 0
        best_color = 'unknown'

        for label, (lo, hi) in COLOR_RANGES.items():
            if 'orange' in label:
                continue
            mask = cv2.inRange(hsv_search, np.array(lo), np.array(hi))
            if label == 'red1':
                mask2 = cv2.inRange(hsv_search,
                                     np.array(COLOR_RANGES['red2'][0]),
                                     np.array(COLOR_RANGES['red2'][1]))
                mask = cv2.bitwise_or(mask, mask2)
                label = 'red'
            elif label == 'red2':
                continue  # handled with red1

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area > self._min_area and area > best_area:
                    best_area = area
                    best_contour = c
                    best_color = label

        if best_contour is None:
            return {'type': 'detection', 'color': 'unknown',
                    'shape': 'unknown', 'distance_m': -1.0,
                    'status': 'nothing_found'}

        shape = _classify_shape(best_contour)
        dist = self._estimate_distance(best_contour, bgr.shape)

        return {
            'type': 'detection',
            'color': best_color,
            'shape': shape,
            'distance_m': round(dist, 2),
            'status': 'ok',
        }

    def _estimate_distance(self, contour, img_shape) -> float:
        """Estimate object distance.

        Priority order:
        1. Depth image (OAK-D stereo) — median of valid pixels inside bounding box
        2. LiDAR bearing match — minimum range in the object's angular sector
        3. Pinhole camera formula — using contour bounding-box height (fallback)
        """
        h_img, w_img = img_shape[:2]
        x, y, w, h = cv2.boundingRect(contour)
        cx_px = x + w / 2.0

        # --- 1. Depth image (OAK-D) ---
        if self._latest_depth is not None:
            dh, dw = self._latest_depth.shape[:2]
            # Scale bounding box to depth image dimensions (may differ from RGB)
            sx = dw / w_img
            sy = dh / h_img
            dx = int(x * sx)
            dy = int(y * sy)
            dw_box = max(1, int(w * sx))
            dh_box = max(1, int(h * sy))
            # Clamp to depth image bounds
            dx2 = min(dx + dw_box, dw)
            dy2 = min(dy + dh_box, dh)
            roi = self._latest_depth[dy:dy2, dx:dx2]
            valid = roi[(roi > 0.1) & (roi < 30.0)]   # 0.1–30 m valid range
            if valid.size > 0:
                return float(np.median(valid))

        # --- 2. LiDAR-based distance ---
        if self._latest_scan is not None and self._camera_info is not None:
            scan = self._latest_scan
            fx = self._camera_info.k[0]  # focal length in x (pixels)
            if fx > 0:
                cx_norm = (cx_px - w_img / 2.0) / fx
                bearing = math.atan(cx_norm)
                hw_norm = (w / 2.0) / fx
                half_ang = math.atan(hw_norm)
                min_dist = float('inf')
                for i, r in enumerate(scan.ranges):
                    if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                        continue
                    ray_ang = scan.angle_min + i * scan.angle_increment
                    if abs(ray_ang - bearing) < half_ang + 0.05:
                        min_dist = min(min_dist, r)
                if min_dist < float('inf'):
                    return min_dist

        # --- 3. Pinhole fallback ---
        if self._camera_info is not None and h > 0:
            fy = self._camera_info.k[4]
            if fy > 0:
                return (self._cone_h * fy) / h

        return -1.0

    def get_detections(self) -> list[dict]:
        return list(self._detections)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
