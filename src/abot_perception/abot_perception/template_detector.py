"""Bounded legacy find_object_2d image matching with local SIFT features."""

import math
from numbers import Integral
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from .detection_utils import ImageBox, ImageDetector, spin_detector


MAX_TEMPLATE_IDS = 8


def _normalize_template_id(value, parameter_name):
    if isinstance(value, bool):
        raise ValueError(f'{parameter_name} must contain positive numeric legacy asset IDs')
    if isinstance(value, Integral):
        value = str(value)
    if (not isinstance(value, str) or not value.isdigit() or
            str(int(value)) != value or int(value) <= 0):
        raise ValueError(f'{parameter_name} must contain positive numeric legacy asset IDs')
    return value


def resolve_template_ids(template_ids, template_id='10'):
    """Resolve the bounded multi-ID option, falling back to the legacy ID."""
    if not isinstance(template_ids, str):
        raise ValueError('template_ids must be a comma-separated string')
    if template_ids == '':
        return (_normalize_template_id(template_id, 'template_id'),)

    values = template_ids.split(',')
    if len(values) > MAX_TEMPLATE_IDS:
        raise ValueError(f'template_ids may contain at most {MAX_TEMPLATE_IDS} IDs')
    ids = []
    for value in values:
        token = value.strip()
        if not token:
            raise ValueError('template_ids must not contain empty or whitespace-only IDs')
        ids.append(_normalize_template_id(token, 'template_ids'))
    if len(set(ids)) != len(ids):
        raise ValueError('template_ids must not contain duplicate IDs')
    return tuple(ids)


class TemplateMatcher:
    def __init__(self, template_gray, template_id, min_matches=12, min_inliers=8,
                 feature=None):
        if isinstance(template_id, bool):
            raise ValueError('Invalid template matcher configuration')
        template_id = str(template_id)
        if (not isinstance(template_gray, np.ndarray) or
                template_gray.dtype != np.uint8 or template_gray.ndim != 2 or
                template_gray.size == 0 or not template_id.isdigit() or
                str(int(template_id)) != template_id or int(template_id) <= 0 or
                not isinstance(min_matches, Integral) or isinstance(min_matches, bool) or
                not isinstance(min_inliers, Integral) or isinstance(min_inliers, bool) or
                min_matches < 4 or min_inliers < 4 or min_inliers > min_matches):
            raise ValueError('Invalid template matcher configuration')
        if not hasattr(cv2, 'SIFT_create'):
            raise ValueError('OpenCV SIFT is required for these low-texture templates')
        self.template_id = str(template_id)
        self.template_shape = template_gray.shape
        self.min_matches = min_matches
        self.min_inliers = min_inliers
        self.feature = feature if feature is not None else cv2.SIFT_create(nfeatures=1200)
        self.keypoints, self.descriptors = self.feature.detectAndCompute(template_gray, None)
        if self.descriptors is None or len(self.keypoints) < min_matches:
            raise ValueError(f'Template {template_id} has too few features')
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)

    def detect(self, image_bgr):
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0:
            raise ValueError('Expected a nonempty BGR image')
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.feature.detectAndCompute(gray, None)
        return self.detect_features(gray.shape, keypoints, descriptors)

    def detect_features(self, image_shape, keypoints, descriptors):
        """Match precomputed frame features; lets a template bank share SIFT work."""
        if descriptors is None or len(keypoints) < self.min_matches:
            return []
        pairs = self.matcher.knnMatch(self.descriptors, descriptors, k=2)
        good = [first for pair in pairs if len(pair) == 2
                for first, second in [pair] if first.distance < 0.7 * second.distance]
        if len(good) < self.min_matches:
            return []
        source = np.float32([self.keypoints[item.queryIdx].pt for item in good])
        destination = np.float32([keypoints[item.trainIdx].pt for item in good])
        transform, mask = cv2.findHomography(source, destination, cv2.RANSAC, 3.0)
        if transform is None or mask is None or int(mask.sum()) < self.min_inliers:
            return []
        template_h, template_w = self.template_shape
        outline = np.float32([[0, 0], [template_w - 1, 0],
                              [template_w - 1, template_h - 1], [0, template_h - 1]])
        projected = cv2.perspectiveTransform(outline.reshape(1, -1, 2), transform)[0]
        height, width = image_shape[:2]
        if (not np.isfinite(projected).all() or
                np.any(projected[:, 0] < 0) or np.any(projected[:, 0] >= width) or
                np.any(projected[:, 1] < 0) or np.any(projected[:, 1] >= height) or
                not cv2.isContourConvex(projected) or
                cv2.contourArea(projected) < self.min_matches * self.min_matches):
            return []
        x0, y0 = projected.min(axis=0)
        x1, y1 = projected.max(axis=0)
        if not all(math.isfinite(float(v)) for v in (x0, y0, x1, y1)):
            return []
        return [ImageBox(f'template:{self.template_id}', float((x0 + x1) / 2),
                         float((y0 + y1) / 2), float(x1 - x0), float(y1 - y0),
                         float(mask.sum() / len(good)))]


class TemplateBank:
    """Precomputed template descriptors with one SIFT extraction per input frame."""

    def __init__(self, template_dir, template_ids, min_matches=12, min_inliers=8,
                 feature=None):
        self.template_ids = tuple(template_ids)
        self.feature = feature if feature is not None else cv2.SIFT_create(nfeatures=1200)
        self.matchers = []
        for template_id in self.template_ids:
            image_path = Path(template_dir) / f'{template_id}.png'
            template = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if template is None:
                raise ValueError(f'Template image unavailable: {image_path}')
            self.matchers.append(TemplateMatcher(
                template, template_id, min_matches, min_inliers, feature=self.feature))

    def detect(self, image_bgr):
        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.size == 0:
            raise ValueError('Expected a nonempty BGR image')
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.feature.detectAndCompute(gray, None)
        boxes = []
        for matcher in self.matchers:
            boxes.extend(matcher.detect_features(gray.shape, keypoints, descriptors))
        return boxes


class TemplateDetector(ImageDetector):
    def __init__(self, parameter_overrides=None):
        super().__init__('template_detector', '/perception/templates', 5.0,
                         parameter_overrides=parameter_overrides)
        default_dir = str(Path(get_package_share_directory('abot_perception')) / 'templates')
        self.declare_parameter('template_dir', default_dir)
        self.declare_parameter('template_id', '10')
        self.declare_parameter('template_ids', '')
        self.declare_parameter('min_matches', 12)
        self.declare_parameter('min_inliers', 8)
        template_ids = resolve_template_ids(
            self.get_parameter('template_ids').value,
            self.get_parameter('template_id').value)
        template_dir = str(self.get_parameter('template_dir').value).strip()
        if not template_dir:
            raise ValueError('template_dir must not be empty')
        self.template_ids = template_ids
        self.template_bank = TemplateBank(
            template_dir, template_ids,
            int(self.get_parameter('min_matches').value),
            int(self.get_parameter('min_inliers').value))

    def find_boxes(self, image_bgr):
        return self.template_bank.detect(image_bgr)


def main():
    spin_detector(TemplateDetector)
