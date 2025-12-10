---
sidebar_position: 5
title: "Computer Vision"
description: "Real-time object detection with YOLO, semantic segmentation, 6D pose estimation, and vision pipeline integration for humanoid robots"
keywords: [computer vision, yolo, object detection, semantic segmentation, pose estimation, opencv, pytorch, ros2]
---

# Chapter 5: Computer Vision

## Introduction

**Computer vision** enables humanoid robots to perceive and understand their environment visually. For manipulation and navigation, robots need to:

1. **Detect objects**: Identify what objects are present (classification + localization)
2. **Segment scenes**: Understand spatial layout and surfaces
3. **Estimate poses**: Determine 6D pose (position + orientation) for grasping
4. **Track motion**: Follow objects over time for dynamic interaction

This chapter covers state-of-the-art deep learning methods deployed in production humanoid systems, with emphasis on **real-time performance** (30+ FPS on edge devices like Jetson AGX Orin).

**Learning Objectives**:
- Deploy YOLO for real-time object detection in ROS 2
- Implement semantic segmentation for scene understanding
- Estimate 6D object poses for robotic manipulation
- Build end-to-end vision pipelines with performance profiling
- Integrate vision with motion planning and control

## 5.1 Object Detection with YOLO

### 5.1.1 YOLO Architecture Overview

**YOLO (You Only Look Once)** is a single-stage object detector that treats detection as a regression problem. Unlike two-stage detectors (R-CNN family), YOLO processes the entire image in one forward pass, achieving real-time performance.

**YOLOv8** (2023, Ultralytics) improvements:
- **Anchor-free** design (simplifies deployment)
- **CSPDarknet backbone** for efficient feature extraction
- **PANet neck** for multi-scale feature fusion
- **Decoupled head** for classification and localization

**Architecture**:
```
Input (640x640x3)
   ↓
Backbone (CSPDarknet53)
   ↓ (P3, P4, P5 feature maps)
Neck (PANet)
   ↓ (Multi-scale features)
Head (Detection head)
   ↓
Outputs: [boxes, scores, classes]
```

**Output Format**: For each detection:
- **Bounding box**: $(x_{center}, y_{center}, width, height)$ in normalized coordinates [0, 1]
- **Confidence score**: Probability that box contains an object
- **Class probabilities**: Distribution over object classes

### 5.1.2 YOLOv8 Deployment with PyTorch

**Installation**:
```bash
pip install ultralytics opencv-python-headless
```

**Python Implementation**:

```python
import cv2
import numpy as np
from ultralytics import YOLO
from typing import List, Tuple
import time

class YOLODetector:
    """
    YOLOv8 object detector with performance profiling.
    """
    def __init__(self, model_path: str = 'yolov8n.pt', confidence_threshold: float = 0.5):
        """
        Args:
            model_path: Path to YOLOv8 weights (n=nano, s=small, m=medium, l=large, x=extra-large)
            confidence_threshold: Minimum confidence for detections
        """
        self.model = YOLO(model_path)
        self.conf_threshold = confidence_threshold

        # Class names (COCO dataset)
        self.class_names = self.model.names

        # Performance tracking
        self.inference_times = []

    def detect(self, image: np.ndarray) -> List[dict]:
        """
        Run detection on image.

        Args:
            image: Input image (H x W x 3, BGR format)

        Returns:
            List of detections, each dict with:
                - 'box': [x_min, y_min, x_max, y_max]
                - 'confidence': float
                - 'class_id': int
                - 'class_name': str
        """
        start_time = time.time()

        # Run inference
        results = self.model(image, conf=self.conf_threshold, verbose=False)

        inference_time = time.time() - start_time
        self.inference_times.append(inference_time)

        # Parse results
        detections = []
        for result in results:
            boxes = result.boxes

            for i in range(len(boxes)):
                box = boxes.xyxy[i].cpu().numpy()  # [x_min, y_min, x_max, y_max]
                conf = float(boxes.conf[i].cpu().numpy())
                class_id = int(boxes.cls[i].cpu().numpy())

                detections.append({
                    'box': box,
                    'confidence': conf,
                    'class_id': class_id,
                    'class_name': self.class_names[class_id]
                })

        return detections

    def visualize(self, image: np.ndarray, detections: List[dict]) -> np.ndarray:
        """
        Draw bounding boxes on image.

        Args:
            image: Input image
            detections: List of detections from detect()

        Returns:
            Annotated image
        """
        vis_image = image.copy()

        for det in detections:
            x_min, y_min, x_max, y_max = det['box'].astype(int)
            class_name = det['class_name']
            confidence = det['confidence']

            # Draw box
            cv2.rectangle(vis_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(vis_image, (x_min, y_min - label_h - 10), (x_min + label_w, y_min), (0, 255, 0), -1)
            cv2.putText(vis_image, label, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return vis_image

    def get_fps(self) -> float:
        """Get average FPS over all inferences."""
        if not self.inference_times:
            return 0.0
        return 1.0 / np.mean(self.inference_times)


# Example usage
if __name__ == "__main__":
    # Initialize detector (yolov8n = nano model, fastest)
    detector = YOLODetector('yolov8n.pt', confidence_threshold=0.5)

    # Load test image
    image_path = 'test_image.jpg'  # Replace with your image
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not load image from {image_path}")
        exit(1)

    print(f"Image shape: {image.shape}")

    # Detect objects
    print("Running detection...")
    detections = detector.detect(image)

    print(f"\n=== Detection Results ===")
    print(f"Found {len(detections)} objects")
    for i, det in enumerate(detections):
        print(f"{i+1}. {det['class_name']}: {det['confidence']:.3f}")

    # Visualize
    vis_image = detector.visualize(image, detections)

    # Performance
    fps = detector.get_fps()
    cv2.putText(vis_image, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Save result
    cv2.imwrite('detection_result.jpg', vis_image)
    print(f"\nVisualization saved to detection_result.jpg")
    print(f"Average FPS: {fps:.1f}")

    # Benchmark
    print("\n=== Benchmarking ===")
    num_trials = 100
    for _ in range(num_trials):
        _ = detector.detect(image)

    final_fps = detector.get_fps()
    print(f"Average FPS over {num_trials} trials: {final_fps:.1f}")
```

**Expected Output** (on RTX 3060):
```
Image shape: (720, 1280, 3)
Running detection...

=== Detection Results ===
Found 5 objects
1. person: 0.893
2. bottle: 0.756
3. cup: 0.634
4. laptop: 0.901
5. mouse: 0.582

Visualization saved to detection_result.jpg
Average FPS: 67.3

=== Benchmarking ===
Average FPS over 100 trials: 71.2
```

### 5.1.3 ROS 2 Integration

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YOLONode(Node):
    """
    ROS 2 node for YOLO object detection.

    Subscribes to: /camera/image_raw
    Publishes: /detections (vision_msgs/Detection2DArray)
               /detections/image (annotated image)
    """
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('publish_viz', True)

        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('confidence_threshold').value
        self.publish_viz = self.get_parameter('publish_viz').value

        # Initialize YOLO
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        if self.publish_viz:
            self.viz_pub = self.create_publisher(Image, '/detections/image', 10)

        self.get_logger().info(f"YOLO detector initialized with model: {model_path}")

    def image_callback(self, msg: Image):
        """Process incoming image and publish detections."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for result in results:
                boxes = result.boxes

                for i in range(len(boxes)):
                    # Parse detection
                    box = boxes.xyxy[i].cpu().numpy()
                    conf = float(boxes.conf[i].cpu().numpy())
                    class_id = int(boxes.cls[i].cpu().numpy())
                    class_name = self.model.names[class_id]

                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = msg.header

                    # Bounding box
                    x_min, y_min, x_max, y_max = box
                    detection.bbox.center.position.x = float((x_min + x_max) / 2)
                    detection.bbox.center.position.y = float((y_min + y_max) / 2)
                    detection.bbox.size_x = float(x_max - x_min)
                    detection.bbox.size_y = float(y_max - y_min)

                    # Class hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(class_id)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)

                    detection_array.detections.append(detection)

            # Publish detections
            self.detection_pub.publish(detection_array)

            # Publish visualization
            if self.publish_viz and len(detection_array.detections) > 0:
                annotated_image = result[0].plot()  # YOLO built-in visualization
                viz_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run**:
```bash
# Terminal 1: Camera driver
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: YOLO detector
ros2 run your_package yolo_node.py --ros-args -p model_path:=yolov8n.pt -p confidence_threshold:=0.5

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /detections/image
```

## 5.2 Semantic Segmentation

### 5.2.1 DeepLabV3+ for Scene Segmentation

**Semantic segmentation** assigns a class label to each pixel, enabling understanding of surfaces, obstacles, and traversable areas.

**DeepLabV3+** architecture:
- **Encoder**: ResNet or MobileNet backbone
- **ASPP (Atrous Spatial Pyramid Pooling)**: Multi-scale context
- **Decoder**: Recovers spatial resolution

**PyTorch Implementation**:

```python
import torch
import torchvision.transforms as T
from torchvision import models
import cv2
import numpy as np

class SemanticSegmentation:
    """
    DeepLabV3+ semantic segmentation with COCO-trained weights.
    """
    def __init__(self, device: str = 'cuda'):
        self.device = device

        # Load pretrained DeepLabV3 (trained on COCO+Cityscapes)
        self.model = models.segmentation.deeplabv3_resnet101(pretrained=True)
        self.model = self.model.to(device)
        self.model.eval()

        # Preprocessing
        self.transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # COCO class names (21 classes)
        self.classes = [
            'background', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow'
        ]

    def segment(self, image: np.ndarray) -> np.ndarray:
        """
        Segment image into classes.

        Args:
            image: Input image (H x W x 3, RGB)

        Returns:
            Segmentation mask (H x W), values are class IDs
        """
        # Preprocess
        input_tensor = self.transform(image).unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad():
            output = self.model(input_tensor)['out'][0]

        # Argmax to get class per pixel
        mask = output.argmax(0).cpu().numpy()

        return mask

    def visualize(self, image: np.ndarray, mask: np.ndarray, alpha: float = 0.5) -> np.ndarray:
        """
        Overlay segmentation mask on image with color coding.

        Args:
            image: Original image
            mask: Segmentation mask
            alpha: Transparency of overlay

        Returns:
            Blended image
        """
        # Generate color map
        np.random.seed(42)
        colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype=np.uint8)
        colors[0] = [0, 0, 0]  # Background = black

        # Create colored mask
        colored_mask = colors[mask]

        # Blend
        blended = cv2.addWeighted(image, 1 - alpha, colored_mask, alpha, 0)

        return blended


# Example
if __name__ == "__main__":
    segmenter = SemanticSegmentation(device='cuda' if torch.cuda.is_available() else 'cpu')

    image = cv2.imread('test_image.jpg')
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    mask = segmenter.segment(image_rgb)

    # Visualize
    vis = segmenter.visualize(image, mask, alpha=0.6)
    cv2.imwrite('segmentation_result.jpg', vis)

    print(f"Unique classes in image: {np.unique(mask)}")
    for class_id in np.unique(mask):
        if class_id < len(segmenter.classes):
            print(f"  - {segmenter.classes[class_id]}")
```

## 5.3 6D Pose Estimation

### 5.3.1 PVN3D for 6D Object Pose

**6D pose estimation** determines the full 6-DOF transformation (3D position + 3D orientation) of objects relative to the camera. This is essential for robotic grasping.

**Methods**:
1. **PnP-based**: Detect 2D keypoints, match to 3D model, solve PnP
2. **Dense prediction**: Predict 3D coordinates for each pixel (e.g., PVN3D, DOPE)
3. **Template matching**: Compare rendered views to image

**Simplified PnP Approach**:

```python
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

class PoseEstimator:
    """
    6D pose estimation using PnP with detected 2D keypoints.
    """
    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """
        Args:
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: Distortion coefficients
        """
        self.K = camera_matrix
        self.dist_coeffs = dist_coeffs

    def estimate_pose(
        self,
        object_points_3d: np.ndarray,
        image_points_2d: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Estimate 6D pose using PnP.

        Args:
            object_points_3d: 3D points in object frame (N x 3)
            image_points_2d: Corresponding 2D points in image (N x 2)

        Returns:
            (rvec, tvec): Rotation vector and translation vector
        """
        success, rvec, tvec = cv2.solvePnP(
            object_points_3d,
            image_points_2d,
            self.K,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            raise ValueError("PnP failed to find solution")

        return rvec, tvec

    def pose_to_matrix(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        """Convert (rvec, tvec) to 4x4 transformation matrix."""
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T


# Example: Estimate pose of known object
if __name__ == "__main__":
    # Camera intrinsics (example for RealSense D435i)
    fx, fy = 615.0, 615.0  # Focal lengths
    cx, cy = 320.0, 240.0  # Principal point
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist_coeffs = np.zeros(5)

    estimator = PoseEstimator(K, dist_coeffs)

    # Known 3D points on object (e.g., corners of a box)
    # Units: meters, origin at object center
    object_points_3d = np.array([
        [-0.05, -0.05, 0],
        [0.05, -0.05, 0],
        [0.05, 0.05, 0],
        [-0.05, 0.05, 0]
    ], dtype=np.float32)

    # Detected 2D points in image (from keypoint detector)
    image_points_2d = np.array([
        [200, 150],
        [400, 150],
        [400, 300],
        [200, 300]
    ], dtype=np.float32)

    # Estimate pose
    rvec, tvec = estimator.estimate_pose(object_points_3d, image_points_2d)

    # Convert to matrix
    T = estimator.pose_to_matrix(rvec, tvec)

    print("6D Pose Estimate:")
    print("Rotation (Euler angles, degrees):")
    euler = Rotation.from_rotvec(rvec.flatten()).as_euler('xyz', degrees=True)
    print(f"  Roll: {euler[0]:.2f}°, Pitch: {euler[1]:.2f}°, Yaw: {euler[2]:.2f}°")
    print("Translation (meters):")
    print(f"  X: {tvec[0,0]:.3f}, Y: {tvec[1,0]:.3f}, Z: {tvec[2,0]:.3f}")
```

## 5.4 Real-Time Vision Pipeline

### 5.4.1 Pipeline Architecture

```
Camera → Undistortion → Detection → Segmentation → Pose Estimation → ROS 2 Topics
  (30 Hz)    (GPU)        (YOLO)     (DeepLab)      (PnP)         (TF2 transforms)
```

**Performance Budget** (for 30 FPS = 33ms total):
- Image acquisition: 2ms
- Undistortion: 1ms
- Detection: 15ms
- Segmentation: 10ms (optional)
- Pose estimation: 3ms
- Publishing: 2ms

### 5.4.2 Optimized Pipeline

```python
import threading
import queue

class VisionPipeline:
    """
    Multi-threaded vision pipeline for real-time processing.
    """
    def __init__(self, detector, segmenter, pose_estimator):
        self.detector = detector
        self.segmenter = segmenter
        self.pose_estimator = pose_estimator

        self.image_queue = queue.Queue(maxsize=2)
        self.result_queue = queue.Queue(maxsize=2)

        self.running = False

    def start(self):
        """Start processing threads."""
        self.running = True
        self.thread = threading.Thread(target=self._process_loop)
        self.thread.start()

    def stop(self):
        """Stop processing."""
        self.running = False
        self.thread.join()

    def submit_image(self, image: np.ndarray):
        """Submit image for processing (non-blocking)."""
        try:
            self.image_queue.put_nowait(image)
        except queue.Full:
            pass  # Drop frame if queue full

    def get_result(self, timeout: float = 0.01):
        """Get latest result (non-blocking)."""
        try:
            return self.result_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _process_loop(self):
        """Main processing loop."""
        while self.running:
            try:
                image = self.image_queue.get(timeout=0.1)

                # Process
                detections = self.detector.detect(image)
                # Optional: segmentation, pose estimation

                # Store result
                result = {'detections': detections}
                self.result_queue.put(result)

            except queue.Empty:
                continue
```

## 5.5 Exercises

### Exercise 1 (Easy): YOLO Custom Classes
Train YOLOv8 on a custom dataset with 3 classes: bottle, cup, book. Use Roboflow for data annotation.

### Exercise 2 (Easy): FPS Benchmark
Compare YOLOv8n, YOLOv8s, YOLOv8m on your hardware. Plot FPS vs. accuracy trade-off.

### Exercise 3 (Medium): Depth-Aware Detection
Integrate depth camera (RealSense) to output 3D bounding boxes in world coordinates.

### Exercise 4 (Medium): Object Tracking
Implement ByteTrack algorithm to assign consistent IDs to detected objects across frames.

### Exercise 5 (Hard): Grasp Pose Estimation
Combine 6D pose with object mesh to compute grasp poses using GraspNet.

## Summary

This chapter covered:
1. **YOLO** for real-time object detection (70+ FPS)
2. **Semantic segmentation** for scene understanding
3. **6D pose estimation** for manipulation
4. **Vision pipelines** with performance optimization

**Next Chapter**: [Capstone Project](./capstone.md) - Integrate all modules

## References

1. Redmon, J., et al. (2016). "You Only Look Once: Unified, Real-Time Object Detection". *CVPR*.
2. Chen, L. C., et al. (2018). "Encoder-Decoder with Atrous Separable Convolution for Semantic Image Segmentation". *ECCV*.
3. Wang, C., et al. (2021). "PVN3D: A Deep Point-wise 3D Keypoints Voting Network for 6DoF Pose Estimation". *CVPR*.
4. Ultralytics YOLOv8: https://docs.ultralytics.com/
