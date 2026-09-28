"""ROS 2 migration of the old OpenCV HOG whole-person experiment."""

import math

import cv2
import numpy as np

from .detection_utils import ImageBox, ImageDetector, spin_detector


def detect_people(image_bgr, hog, min_weight=0.0, min_height_px=64.0):
    if (not isinstance(image_bgr, np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0):
        raise ValueError('Expected a nonempty BGR image')
    if (hog is None or isinstance(min_weight, bool) or isinstance(min_height_px, bool) or
            not isinstance(min_weight, (int, float)) or
            not isinstance(min_height_px, (int, float)) or
            not math.isfinite(min_weight) or not math.isfinite(min_height_px) or
            min_weight < 0 or min_height_px <= 0):
        raise ValueError('Invalid person filter')
    rectangles, weights = hog.detectMultiScale(
        image_bgr, winStride=(8, 8), padding=(8, 8), scale=1.05)
    height, width = image_bgr.shape[:2]
    boxes = []
    for (x, y, w, h), weight in zip(rectangles, weights):
        if (not all(math.isfinite(float(value)) for value in (x, y, w, h, weight)) or
                float(weight) < min_weight or h < min_height_px or w <= 0 or x < 0 or y < 0 or
                x + w > width or y + h > height):
            continue
        # HOG weight is a detector margin, not a calibrated probability.
        boxes.append(ImageBox('person', x + w / 2.0, y + h / 2.0,
                              float(w), float(h),
                              min(1.0, max(0.0, float(weight)))))
    return boxes


class PersonDetector(ImageDetector):
    def __init__(self):
        super().__init__('person_detector', '/perception/people', 3.0)
        self.declare_parameter('min_weight', 0.0)
        self.declare_parameter('min_height_px', 64.0)
        self.min_weight = float(self.get_parameter('min_weight').value)
        self.min_height_px = float(self.get_parameter('min_height_px').value)
        if (not math.isfinite(self.min_weight) or self.min_weight < 0 or
                not math.isfinite(self.min_height_px) or self.min_height_px <= 0):
            raise ValueError('Invalid person detector thresholds')
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def find_boxes(self, image_bgr):
        return detect_people(image_bgr, self.hog, self.min_weight,
                             self.min_height_px)


def main():
    spin_detector(PersonDetector)
