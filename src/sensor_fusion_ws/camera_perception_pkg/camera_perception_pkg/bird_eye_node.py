"""Show lane segmentation from one camera as bird's-eye and original views."""

import os
from typing import Dict, Sequence

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO


class BirdEyeNode(Node):
    """Detect lanes from one camera and visualise the drivable centre lines."""

    def __init__(self) -> None:
        super().__init__('bird_eye_node')

        self.declare_parameter('input_topic', 'image_raw')
        self.declare_parameter('output_topic', 'bird_eye/image')
        self.declare_parameter('roi_topic', 'bird_eye/roi')
        self.declare_parameter('comparison_topic', 'bird_eye/comparison')
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 640)
        self.declare_parameter('destination_margin', 0)
        # Empty means the lane segmentation model shipped with this package.
        self.declare_parameter('model', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('show_camera_background', True)
        self.declare_parameter('line_thickness', 3)
        self.declare_parameter('centerline_thickness', 4)
        self.declare_parameter('centerline_sample_step', 12)
        # OpenCV uses BGR.
        self.declare_parameter('lane_1_color_bgr', [0, 255, 0])
        self.declare_parameter('lane_2_color_bgr', [0, 165, 255])
        self.declare_parameter('centerline_color_bgr', [0, 255, 255])
        self.declare_parameter('roi_color_bgr', [255, 120, 0])
        self.declare_parameter('roi_overlay_alpha', 0.25)
        # Four road-plane corners: top-left, top-right, bottom-right, bottom-left.
        self.declare_parameter(
            'src_points', [0.30, 0.56, 0.70, 0.56, 1.00, 1.00, 0.00, 1.00])
        self.declare_parameter('normalized_src_points', True)
        # The requested single side-by-side OpenCV window is enabled by default.
        self.declare_parameter('show_preview', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.roi_topic = self.get_parameter('roi_topic').value
        self.comparison_topic = self.get_parameter('comparison_topic').value
        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.destination_margin = int(
            self.get_parameter('destination_margin').value)
        self.model_path = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.show_camera_background = bool(
            self.get_parameter('show_camera_background').value)
        self.line_thickness = int(self.get_parameter('line_thickness').value)
        self.centerline_thickness = int(
            self.get_parameter('centerline_thickness').value)
        self.centerline_sample_step = int(
            self.get_parameter('centerline_sample_step').value)
        self.lane_1_color = tuple(self.get_parameter('lane_1_color_bgr').value)
        self.lane_2_color = tuple(self.get_parameter('lane_2_color_bgr').value)
        self.centerline_color = tuple(
            self.get_parameter('centerline_color_bgr').value)
        self.roi_color = tuple(self.get_parameter('roi_color_bgr').value)
        self.roi_overlay_alpha = float(
            self.get_parameter('roi_overlay_alpha').value)
        self.src_points = list(self.get_parameter('src_points').value)
        self.normalized_src_points = bool(
            self.get_parameter('normalized_src_points').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)

        self._validate_parameters()
        self.bridge = CvBridge()
        self._last_input_size = None
        self._homography = None
        self._source_polygon = None
        self._source_roi_mask = None
        self.model = self._load_model()

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, self.output_topic, qos)
        self.roi_publisher = self.create_publisher(Image, self.roi_topic, qos)
        self.comparison_publisher = self.create_publisher(
            Image, self.comparison_topic, qos)
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos)
        self.get_logger().info(
            f'Single-camera lane bird-eye: {self.input_topic} -> '
            f'{self.output_topic}, {self.roi_topic}, {self.comparison_topic}')

    def _validate_parameters(self) -> None:
        if len(self.src_points) != 8:
            raise ValueError('src_points must contain exactly 8 values (4 x,y pairs).')
        if self.output_width <= 0 or self.output_height <= 0:
            raise ValueError('output_width and output_height must be positive.')
        if not 0 <= self.destination_margin < min(self.output_width,
                                                  self.output_height) / 2:
            raise ValueError('destination_margin is outside the output image.')
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be 0.0 to 1.0.')
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError('iou_threshold must be 0.0 to 1.0.')
        if self.line_thickness < 1 or self.centerline_thickness < 1:
            raise ValueError('Line thicknesses must be at least 1.')
        if self.centerline_sample_step < 2:
            raise ValueError('centerline_sample_step must be at least 2.')
        if not 0.0 <= self.roi_overlay_alpha <= 1.0:
            raise ValueError('roi_overlay_alpha must be 0.0 to 1.0.')
        for color in (self.lane_1_color, self.lane_2_color,
                      self.centerline_color, self.roi_color):
            if len(color) != 3 or any(not 0 <= value <= 255 for value in color):
                raise ValueError('Colours must be BGR arrays of three 0-255 values.')

    def _load_model(self) -> YOLO:
        if not self.model_path:
            self.model_path = os.path.join(
                get_package_share_directory('camera_perception_pkg'),
                'models', 'lane_seg.pt')
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f'Lane model was not found: {self.model_path}')

        try:
            model = YOLO(self.model_path)
        except Exception as error:
            raise RuntimeError(f'Cannot load lane model: {error}') from error
        if model.task != 'segment':
            raise ValueError(
                f'Lane model must be a segmentation model, got {model.task!r}.')
        self.get_logger().info(
            f'Loaded lane model: {self.model_path}; classes: {model.names}')
        return model

    def _make_homography(self, width: int, height: int) -> None:
        src = np.asarray(self.src_points, dtype=np.float32).reshape(4, 2)
        if self.normalized_src_points:
            src *= np.array([width - 1, height - 1], dtype=np.float32)

        if (np.any(src[:, 0] < 0) or np.any(src[:, 0] >= width) or
                np.any(src[:, 1] < 0) or np.any(src[:, 1] >= height)):
            raise ValueError(
                f'src_points are outside the {width}x{height} input image.')
        if abs(cv2.contourArea(src)) < 1.0:
            raise ValueError('src_points must form a non-zero road ROI.')

        margin = self.destination_margin
        dst = np.float32([
            [margin, margin],
            [self.output_width - 1 - margin, margin],
            [self.output_width - 1 - margin, self.output_height - 1 - margin],
            [margin, self.output_height - 1 - margin],
        ])
        self._homography = cv2.getPerspectiveTransform(src, dst)
        self._source_polygon = np.rint(src).astype(np.int32)
        self._source_roi_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(self._source_roi_mask, [self._source_polygon], 255)
        self._last_input_size = (width, height)
        self.get_logger().info(
            f'Perspective ROI set for {width}x{height}: {src.tolist()}')

    def _color_for_class(self, class_id: int) -> tuple:
        if class_id == 0:
            return tuple(int(value) for value in self.lane_1_color)
        if class_id == 1:
            return tuple(int(value) for value in self.lane_2_color)
        return (255, 0, 255)

    def _class_name(self, class_id: int) -> str:
        names = self.model.names
        if isinstance(names, dict):
            return str(names.get(class_id, f'lane_{class_id + 1}'))
        if 0 <= class_id < len(names):
            return str(names[class_id])
        return f'lane_{class_id + 1}'

    def _infer_bird_eye_masks(self, frame: np.ndarray) -> Dict[int, np.ndarray]:
        """Infer in camera coordinates, clip to the ROI, then warp each class."""
        result = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            retina_masks=True,
            verbose=False,
        )[0]

        if result.masks is None or result.boxes is None:
            return {}

        masks = result.masks.data.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        input_height, input_width = frame.shape[:2]
        bird_eye_masks = {}

        for mask, class_id in zip(masks, class_ids):
            if mask.shape != (input_height, input_width):
                mask = cv2.resize(mask, (input_width, input_height),
                                  interpolation=cv2.INTER_NEAREST)
            source_mask = (mask > 0.5).astype(np.uint8) * 255
            source_mask = cv2.bitwise_and(source_mask, self._source_roi_mask)
            bird_eye_mask = cv2.warpPerspective(
                source_mask, self._homography,
                (self.output_width, self.output_height),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT)
            if not np.any(bird_eye_mask):
                continue
            if class_id in bird_eye_masks:
                bird_eye_masks[class_id] = cv2.bitwise_or(
                    bird_eye_masks[class_id], bird_eye_mask)
            else:
                bird_eye_masks[class_id] = bird_eye_mask
        return bird_eye_masks

    def _centreline_points(self, mask: np.ndarray) -> np.ndarray:
        """Return a smooth centre line through horizontal slices of a lane mask."""
        occupied_rows = np.flatnonzero(np.any(mask > 0, axis=1))
        if occupied_rows.size < 2:
            return np.empty((0, 1, 2), dtype=np.int32)

        first_y = int(occupied_rows[0])
        last_y = int(occupied_rows[-1])
        samples = []
        step = self.centerline_sample_step
        for start_y in range(first_y, last_y + 1, step):
            end_y = min(start_y + step, last_y + 1)
            row_centres = []
            sampled_rows = []
            for y in range(start_y, end_y):
                xs = np.flatnonzero(mask[y] > 0)
                if xs.size:
                    row_centres.append((float(xs[0]) + float(xs[-1])) / 2.0)
                    sampled_rows.append(y)
            if row_centres:
                samples.append((float(np.median(row_centres)),
                                float(np.median(sampled_rows))))

        if len(samples) < 2:
            return np.empty((0, 1, 2), dtype=np.int32)

        centres = np.asarray(samples, dtype=np.float64)
        y_values = centres[:, 1]
        degree = min(2, len(samples) - 1)
        coefficients = np.polyfit(y_values, centres[:, 0], degree)
        smooth_y = np.arange(int(y_values.min()), int(y_values.max()) + 1, 3)
        smooth_x = np.polyval(coefficients, smooth_y)
        smooth_x = np.clip(smooth_x, 0, mask.shape[1] - 1)
        points = np.column_stack((smooth_x, smooth_y))
        return np.rint(points).astype(np.int32).reshape(-1, 1, 2)

    @staticmethod
    def _draw_label(image: np.ndarray, text: str, position: tuple,
                    color: tuple) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.65
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, scale, thickness)
        x = max(0, min(int(position[0]), image.shape[1] - text_width - 8))
        y = max(text_height + 6,
                min(int(position[1]), image.shape[0] - baseline - 5))
        cv2.rectangle(image, (x - 4, y - text_height - 5),
                      (x + text_width + 4, y + baseline + 3), (0, 0, 0), -1)
        cv2.putText(image, text, (x, y), font, scale, color,
                    thickness, cv2.LINE_AA)

    def _draw_lane_result(self, background: np.ndarray,
                          masks: Dict[int, np.ndarray]) -> np.ndarray:
        """Draw unfilled contours, class labels and yellow lane centre lines."""
        output = (background.copy() if self.show_camera_background
                  else np.zeros_like(background))

        for class_id in sorted(masks):
            mask = masks[class_id]
            color = self._color_for_class(class_id)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, color, self.line_thickness)

            centreline = self._centreline_points(mask)
            if len(centreline) >= 2:
                cv2.polylines(
                    output, [centreline], False,
                    tuple(int(value) for value in self.centerline_color),
                    self.centerline_thickness, cv2.LINE_AA)
                label_position = tuple(centreline[0, 0] + np.array([10, 25]))
            else:
                moments = cv2.moments(mask)
                if moments['m00'] <= 0:
                    continue
                label_position = (
                    int(moments['m10'] / moments['m00']),
                    int(moments['m01'] / moments['m00']))
            self._draw_label(
                output, self._class_name(class_id), label_position, color)
        return output

    def _draw_roi_on_original(self, frame: np.ndarray) -> np.ndarray:
        """Show the exact source polygon used to create the bird's-eye ROI."""
        overlay = frame.copy()
        roi_color = tuple(int(value) for value in self.roi_color)
        cv2.fillPoly(overlay, [self._source_polygon], roi_color)
        annotated = cv2.addWeighted(
            overlay, self.roi_overlay_alpha,
            frame, 1.0 - self.roi_overlay_alpha, 0.0)
        cv2.polylines(annotated, [self._source_polygon], True,
                      roi_color, 2, cv2.LINE_AA)
        label_x = int(np.min(self._source_polygon[:, 0])) + 8
        label_y = int(np.min(self._source_polygon[:, 1])) + 28
        self._draw_label(annotated, 'BIRD-EYE ROI',
                         (label_x, label_y), roi_color)
        return annotated

    @staticmethod
    def _fit_to_panel(image: np.ndarray, width: int, height: int) -> np.ndarray:
        """Fit an image into a panel without changing its aspect ratio."""
        image_height, image_width = image.shape[:2]
        scale = min(width / image_width, height / image_height)
        resized_width = max(1, int(round(image_width * scale)))
        resized_height = max(1, int(round(image_height * scale)))
        resized = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        offset_x = (width - resized_width) // 2
        offset_y = (height - resized_height) // 2
        panel[offset_y:offset_y + resized_height,
              offset_x:offset_x + resized_width] = resized
        return panel

    @staticmethod
    def _add_title(image: np.ndarray, title: str) -> np.ndarray:
        titled = image.copy()
        cv2.rectangle(titled, (0, 0), (titled.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(titled, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return titled

    def image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'Unable to convert input image: {error}')
            return

        height, width = frame.shape[:2]
        if self._last_input_size != (width, height):
            self._make_homography(width, height)

        masks = self._infer_bird_eye_masks(frame)
        roi_frame = cv2.bitwise_and(frame, frame, mask=self._source_roi_mask)
        bird_eye_background = cv2.warpPerspective(
            roi_frame, self._homography,
            (self.output_width, self.output_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT)
        bird_eye_view = self._draw_lane_result(bird_eye_background, masks)

        bird_eye_message = self.bridge.cv2_to_imgmsg(
            bird_eye_view, encoding='bgr8')
        bird_eye_message.header = message.header
        self.publisher.publish(bird_eye_message)

        original_view = self._draw_roi_on_original(frame)
        roi_message = self.bridge.cv2_to_imgmsg(original_view, encoding='bgr8')
        roi_message.header = message.header
        self.roi_publisher.publish(roi_message)

        original_panel = self._fit_to_panel(
            original_view, self.output_width, self.output_height)
        comparison = np.hstack((
            self._add_title(bird_eye_view, 'BIRD-EYE VIEW'),
            self._add_title(original_panel, 'ORIGINAL VIEW / ROI'),
        ))
        comparison_message = self.bridge.cv2_to_imgmsg(
            comparison, encoding='bgr8')
        comparison_message.header = message.header
        self.comparison_publisher.publish(comparison_message)

        if self.show_preview:
            cv2.imshow('Lane bird-eye | original', comparison)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Sequence[str] = None) -> None:
    rclpy.init(args=args)
    node = BirdEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
