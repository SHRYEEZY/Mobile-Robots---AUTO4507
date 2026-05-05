#!/usr/bin/env python3
import os
import time

import cv2
import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from ultralytics import YOLO


class GreekLetterSearchNode(Node):
    def __init__(self):
        super().__init__("greek_letter_search_node")

        self.declare_parameter(
            "model_path",
            "/home/shryeezy/runs/classify/alpha_beta_not_letter/weights/best.pt"
        )
        self.declare_parameter(
            "save_dir",
            "/home/shryeezy/greek_search_results"
        )
        self.declare_parameter("search_rate", 1.0)
        self.declare_parameter("confidence_threshold", 0.70)
        self.declare_parameter("use_white_crop", True)

        # Ink gate parameters
        self.declare_parameter("min_ink_ratio", 0.005)
        self.declare_parameter("max_ink_ratio", 0.35)
        self.declare_parameter("min_ink_area", 80)

        self.model_path = str(self.get_parameter("model_path").value)
        self.save_dir = str(self.get_parameter("save_dir").value)
        self.search_rate = float(self.get_parameter("search_rate").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.use_white_crop = bool(self.get_parameter("use_white_crop").value)

        self.min_ink_ratio = float(self.get_parameter("min_ink_ratio").value)
        self.max_ink_ratio = float(self.get_parameter("max_ink_ratio").value)
        self.min_ink_area = float(self.get_parameter("min_ink_area").value)

        os.makedirs(self.save_dir, exist_ok=True)

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model not found: {self.model_path}")

        self.model = YOLO(self.model_path)

        # DepthAI camera setup
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        video_out = cam.requestOutput(
            (1920, 1080),
            type=dai.ImgFrame.Type.BGR888p
        )

        self.q_rgb = video_out.createOutputQueue()
        self.pipeline.start()

        self.timer = self.create_timer(
            1.0 / self.search_rate,
            self.search_once
        )

        self.get_logger().info("Greek letter search node ready")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Saving results to: {self.save_dir}")
        self.get_logger().info(
            f"Ink gate: min_ratio={self.min_ink_ratio}, "
            f"max_ratio={self.max_ink_ratio}, min_area={self.min_ink_area}"
        )

    def crop_white_page(self, frame):
        """
        Finds the white sheet/object using HSV.
        Returns crop and bounding box.
        If no white object is found, returns None.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White = low saturation, high value/brightness
        lower_white = (0, 0, 150)
        upper_white = (180, 80, 255)

        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, None, mask

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < 5000:
            return None, None, mask

        x, y, w, h = cv2.boundingRect(largest)

        margin = 20
        x1 = max(0, x - margin)
        y1 = max(0, y - margin)
        x2 = min(frame.shape[1], x + w + margin)
        y2 = min(frame.shape[0], y + h + margin)

        crop = frame[y1:y2, x1:x2]

        return crop, (x1, y1, x2, y2), mask

    def has_black_ink(self, crop):
        """
        Checks whether the white crop contains enough dark ink.
        Returns:
            has_ink, ink_ratio, ink_mask
        """
        if crop is None or crop.size == 0:
            return False, 0.0, None

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Dark ink becomes white in this binary mask.
        _, ink_mask = cv2.threshold(
            gray_blur,
            130,
            255,
            cv2.THRESH_BINARY_INV
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_OPEN, kernel)
        ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_CLOSE, kernel)

        ink_pixels = cv2.countNonZero(ink_mask)
        total_pixels = crop.shape[0] * crop.shape[1]
        ink_ratio = ink_pixels / total_pixels if total_pixels > 0 else 0.0

        contours, _ = cv2.findContours(
            ink_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        max_area = 0
        if contours:
            max_area = max(cv2.contourArea(c) for c in contours)

        has_ink = (
            ink_ratio >= self.min_ink_ratio and
            ink_ratio <= self.max_ink_ratio and
            max_area >= self.min_ink_area
        )

        return has_ink, ink_ratio, ink_mask

    def extract_ink_roi(self, crop, ink_mask):
        """
        Crops tighter around the black ink.
        This helps YOLO see the letter shape more than the white background.
        """
        if crop is None or ink_mask is None:
            return crop

        contours, _ = cv2.findContours(
            ink_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return crop

        good_contours = [
            c for c in contours
            if cv2.contourArea(c) >= self.min_ink_area
        ]

        if not good_contours:
            return crop

        all_points = np.vstack(good_contours)
        x, y, w, h = cv2.boundingRect(all_points)

        margin = 30
        x1 = max(0, x - margin)
        y1 = max(0, y - margin)
        x2 = min(crop.shape[1], x + w + margin)
        y2 = min(crop.shape[0], y + h + margin)

        return crop[y1:y2, x1:x2]

    def predict_letter(self, image):
        results = self.model.predict(
            image,
            imgsz=224,
            verbose=False
        )

        probs = results[0].probs
        class_id = int(probs.top1)
        confidence = float(probs.top1conf)
        label = results[0].names[class_id]

        return label, confidence

    def classify_result(self, label, confidence):
        """
        Converts model output into robot-friendly messages.
        Expected labels:
            alpha
            beta
            not_letter
        """
        if confidence < self.confidence_threshold:
            return "AWW_unsure", (0, 0, 255), (
                f"AWW — unsure. label={label}, confidence={confidence:.2f}"
            )

        if label == "not_letter":
            return "AWW_not_letter", (0, 0, 255), (
                f"AWW — not a letter. confidence={confidence:.2f}"
            )

        if label == "alpha":
            return "GREAT_alpha_found", (0, 255, 0), (
                f"GREAT — alpha found! confidence={confidence:.2f}"
            )

        if label == "beta":
            return "GREAT_beta_found", (255, 0, 0), (
                f"GREAT — beta found! confidence={confidence:.2f}"
            )

        return f"GREAT_{label}_found", (0, 255, 255), (
            f"GREAT — Greek letter found: {label}, confidence={confidence:.2f}"
        )

    def search_once(self):
        try:
            in_rgb = self.q_rgb.get()
            frame = in_rgb.getCvFrame()
        except Exception as e:
            self.get_logger().error(f"Camera capture failed: {e}")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        annotated = frame.copy()

        # 1. Find white page/object
        if self.use_white_crop:
            page_crop, bbox, white_mask = self.crop_white_page(frame)
        else:
            page_crop = frame
            bbox = None
            white_mask = None

        if page_crop is None:
            result_text = "AWW_no_white_page"
            self.get_logger().info("AWW — no white page/object found")

            cv2.putText(
                annotated,
                result_text,
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3
            )

            full_path = os.path.join(self.save_dir, f"{timestamp}_{result_text}.jpg")
            cv2.imwrite(full_path, annotated)
            return

        # 2. Check black ink exists
        has_ink, ink_ratio, ink_mask = self.has_black_ink(page_crop)

        if not has_ink:
            result_text = f"AWW_white_but_no_ink_{ink_ratio:.4f}"
            self.get_logger().info(
                f"AWW — white object found, but not enough black ink. "
                f"ink_ratio={ink_ratio:.4f}"
            )

            if bbox is not None:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)

            cv2.putText(
                annotated,
                result_text,
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3
            )

            full_path = os.path.join(self.save_dir, f"{timestamp}_{result_text}.jpg")
            crop_path = os.path.join(self.save_dir, f"{timestamp}_crop_no_ink.jpg")
            mask_path = os.path.join(self.save_dir, f"{timestamp}_ink_mask_no_ink.jpg")

            cv2.imwrite(full_path, annotated)
            cv2.imwrite(crop_path, page_crop)
            if ink_mask is not None:
                cv2.imwrite(mask_path, ink_mask)

            return

        # 3. Crop tighter around ink, so model sees symbol more than white page
        image_for_model = self.extract_ink_roi(page_crop, ink_mask)

        # 4. Run YOLO classifier
        label, confidence = self.predict_letter(image_for_model)

        # 5. Convert model result into alpha/beta/not_letter logic
        result_text, colour, log_msg = self.classify_result(label, confidence)

        self.get_logger().info(
            f"{log_msg}, ink_ratio={ink_ratio:.4f}"
        )

        # Save annotated image
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 3)

        cv2.putText(
            annotated,
            f"{result_text} {label} {confidence:.2f} ink={ink_ratio:.3f}",
            (30, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            colour,
            3
        )

        full_path = os.path.join(
            self.save_dir,
            f"{timestamp}_{result_text}_{confidence:.2f}.jpg"
        )
        page_crop_path = os.path.join(
            self.save_dir,
            f"{timestamp}_page_crop_{label}_{confidence:.2f}.jpg"
        )
        ink_roi_path = os.path.join(
            self.save_dir,
            f"{timestamp}_ink_roi_{label}_{confidence:.2f}.jpg"
        )
        ink_mask_path = os.path.join(
            self.save_dir,
            f"{timestamp}_ink_mask_{label}_{confidence:.2f}.jpg"
        )

        cv2.imwrite(full_path, annotated)
        cv2.imwrite(page_crop_path, page_crop)
        cv2.imwrite(ink_roi_path, image_for_model)
        cv2.imwrite(ink_mask_path, ink_mask)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GreekLetterSearchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
