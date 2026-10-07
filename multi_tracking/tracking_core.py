"""ROS-independent port of the detector/tracker in real_time_6_5.py.

The bundled classifier's 13 feature definitions and weights are retained.
See docs/MIGRATION.md for intentional fixes to segmentation and tracking.
"""

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def scan_segments(ranges, angle_min, angle_increment, range_min, range_max,
                  distance_threshold=0.1, min_points=3):
    """Split ordered valid returns; never connect points across an invalid ray."""
    values = np.asarray(ranges, dtype=float)
    if values.ndim != 1:
        raise ValueError('ranges must be one dimensional')
    if not all(math.isfinite(v) for v in
               (angle_min, angle_increment, range_min, range_max)):
        raise ValueError('scan metadata must be finite')
    if angle_increment == 0 or range_min < 0 or range_max <= range_min:
        raise ValueError('invalid scan angle increment or range limits')
    if distance_threshold <= 0 or min_points < 3:
        raise ValueError('distance_threshold must be positive; min_points >= 3')
    if values.size == 0:
        return []
    valid = (np.isfinite(values) & (values > 0)
             & (values >= range_min) & (values <= range_max))
    angles = angle_min + np.arange(values.size) * angle_increment
    safe_ranges = np.where(valid, values, 0.0)
    xy = np.column_stack((np.cos(angles), np.sin(angles))) * safe_ranges[:, None]
    groups = []
    start = None
    for i in range(len(values)):
        if not valid[i]:
            if start is not None:
                groups.append(xy[start:i])
            start = None
        elif start is None:
            start = i
        elif np.linalg.norm(xy[i] - xy[i - 1]) >= distance_threshold:
            groups.append(xy[start:i])
            start = i
    if start is not None:
        groups.append(xy[start:])
    # A 360-degree scan can split a leg at its first/last ray.
    full_circle = abs(abs(angle_increment) * (len(values) - 1) - 2 * math.pi) <= (
        1.5 * abs(angle_increment))
    if (full_circle and len(groups) > 1 and valid[0] and valid[-1]
            and np.linalg.norm(xy[0] - xy[-1]) < distance_threshold):
        groups[0] = np.vstack((groups[-1], groups[0]))
        groups.pop()
    return [g for g in groups if len(g) >= min_points]


def segment_features(xy, preceding_distance, succeeding_distance):
    """Return T1..T13, preserving the feature scales used by the old model."""
    xy = np.asarray(xy, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3 or not np.isfinite(xy).all():
        raise ValueError('features require at least 3 finite 2D points')
    n = len(xy)
    edges = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    circle_a = np.column_stack((-2 * xy, np.ones(n)))
    cx, cy, c = np.linalg.lstsq(circle_a, -np.sum(xy ** 2, axis=1), rcond=None)[0]
    radius = math.sqrt(max(0.0, cx * cx + cy * cy - c))
    residual = radius - np.linalg.norm(xy - [cx, cy], axis=1)
    # Legacy (n,1)-(n,) broadcasting counted each residual n times. Retain
    # that learned scale rather than changing an input to the existing model.
    circularity = n * float(residual @ residual)
    middle = xy[n // 2] if n % 2 else (xy[n // 2 - 1] + xy[n // 2]) / 2
    deviation = np.linalg.norm(xy - middle) / n
    line_a = np.column_stack((xy[:, 0], np.ones(n)))
    slope, intercept = np.linalg.lstsq(line_a, xy[:, 1], rcond=None)[0]
    linearity = np.linalg.norm(slope * xy[:, 0] + intercept - xy[:, 1])
    v1, v2 = xy[:-2] - xy[1:-1], xy[2:] - xy[1:-1]
    da, db = np.linalg.norm(v1, axis=1), np.linalg.norm(v2, axis=1)
    dc = np.linalg.norm(xy[2:] - xy[:-2], axis=1)
    nonzero = (da > 1e-12) & (db > 1e-12) & (dc > 1e-12)
    theta = np.zeros(n - 2)
    curvature = np.zeros(n - 2)
    theta[nonzero] = np.arccos(np.clip(
        np.sum(v1[nonzero] * v2[nonzero], axis=1) / (da[nonzero] * db[nonzero]),
        -1.0, 1.0))
    # 2*abs(cross) = 4*triangle area, avoids a negative Heron radicand.
    cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
    curvature[nonzero] = 2 * np.abs(cross[nonzero]) / (
        da[nonzero] * db[nonzero] * dc[nonzero])
    features = np.array([
        n, np.linalg.norm(np.std(xy, axis=0)), np.std(edges, ddof=1),
        circularity, radius, np.sum(edges), deviation,
        preceding_distance, succeeding_distance, linearity,
        np.linalg.norm(xy[0] - xy[-1]), np.mean(theta), np.mean(curvature),
    ], dtype=float)
    if not np.isfinite(features).all():
        raise ValueError('non-finite segment features')
    return features


class AdaBoostModel:
    def __init__(self, path):
        data = np.loadtxt(path, ndmin=2)
        if data.shape[0] != 5 or data.shape[1] == 0 or not np.isfinite(data).all():
            raise ValueError('classifier must contain 5 finite rows and at least one column')
        if not np.all((data[0] >= 1) & (data[0] <= 13) & (data[0] == np.floor(data[0]))):
            raise ValueError('classifier feature indices must be integers from 1 to 13')
        if not np.isin(data[4], [0, 1]).all():
            raise ValueError('classifier polarity must be 0 or 1')
        self.feature_indices = data[0].astype(int) - 1
        self.thresholds = data[2]
        self.weights = data[3]
        self.polarities = data[4].astype(bool)

    def score(self, features):
        features = np.asarray(features, dtype=float)
        if features.shape != (13,) or not np.isfinite(features).all():
            raise ValueError('expected 13 finite features')
        values = features[self.feature_indices]
        positive = np.where(self.polarities, values > self.thresholds, values < self.thresholds)
        return float(self.weights @ np.where(positive, 1.0, -1.0))

    def detect(self, segments):
        legs = []
        for i, segment in enumerate(segments):
            preceding = segments[(i - 1) % len(segments)]
            succeeding = segments[(i + 1) % len(segments)]
            features = segment_features(
                segment, np.linalg.norm(segment[0] - preceding[-1]),
                np.linalg.norm(segment[-1] - succeeding[0]))
            if self.score(features) > 0:
                legs.append(segment.mean(axis=0))
        return np.asarray(legs, dtype=float).reshape(-1, 2)


def pair_legs(legs, max_distance=0.8):
    """Greedy closest-pair midpoints, plus unpaired legs for existing tracks."""
    legs = np.asarray(legs, dtype=float).reshape(-1, 2)
    available = list(range(len(legs)))
    pairs = []
    while len(available) >= 2:
        distance, i, j = min(
            (float(np.linalg.norm(legs[a] - legs[b])), a, b)
            for k, a in enumerate(available) for b in available[k + 1:])
        if distance >= max_distance:
            break
        pairs.append((legs[i] + legs[j]) / 2)
        available.remove(i)
        available.remove(j)
    points = pairs + [legs[i].copy() for i in available]
    return np.asarray(points, dtype=float).reshape(-1, 2), len(pairs)


H = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.]])
MEASUREMENT_NOISE = np.array([
    [0.001474613368833, 0.000524673241571],
    [0.000524673241571, 0.001269948264295],
])


@dataclass
class Track:
    id: int
    state: np.ndarray
    covariance: np.ndarray = field(default_factory=lambda: np.eye(4))
    history: deque = field(default_factory=lambda: deque([True] * 20, maxlen=20))
    pair_hits: int = 1

    @property
    def position(self):
        return self.state[[0, 2]].copy()

    def predict(self, dt):
        transition = np.eye(4)
        transition[0, 1] = transition[2, 3] = dt
        acceleration = np.array([[dt ** 2 / 2, 0.], [dt, 0.],
                                 [0., dt ** 2 / 2], [0., dt]])
        self.state = transition @ self.state
        self.covariance = (transition @ self.covariance @ transition.T
                           + 5.0 * acceleration @ acceleration.T)

    def distance(self, point):
        residual = point - H @ self.state
        innovation = H @ self.covariance @ H.T + MEASUREMENT_NOISE
        return max(0.0, float(residual @ np.linalg.solve(innovation, residual)))

    def update(self, point, is_pair):
        innovation = H @ self.covariance @ H.T + MEASUREMENT_NOISE
        gain = np.linalg.solve(innovation, H @ self.covariance).T
        self.state += gain @ (point - H @ self.state)
        correction = np.eye(4) - gain @ H
        self.covariance = (correction @ self.covariance @ correction.T
                           + gain @ MEASUREMENT_NOISE @ gain.T)
        self.covariance = (self.covariance + self.covariance.T) / 2
        self.pair_hits += int(is_pair)


class MultiTracker:
    def __init__(self, pair_distance=0.8, association_gate=250.0):
        if not all(math.isfinite(v) and v > 0 for v in (pair_distance, association_gate)):
            raise ValueError('tracking thresholds must be finite and positive')
        self.pair_distance = pair_distance
        self.association_gate = association_gate
        self.tracks = []
        self.next_id = 0

    def reset(self):
        self.tracks.clear()

    def update(self, legs, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        legs = np.asarray(legs, dtype=float).reshape(-1, 2)
        if not np.isfinite(legs).all():
            raise ValueError('leg coordinates must be finite')
        points, pair_count = pair_legs(legs, self.pair_distance)
        for track in self.tracks:
            track.predict(dt)
        matched = {}
        used = set()
        if self.tracks and len(points):
            costs = np.array([[track.distance(p) for p in points] for track in self.tracks])
            # Each row has a dummy unmatched option; forbidden pairs cannot
            # steal a detection from a valid match. Zero is a valid cost.
            gated = np.where(costs < self.association_gate, costs, self.association_gate * 2)
            augmented = np.column_stack((gated, np.full(
                (len(self.tracks), len(self.tracks)), self.association_gate)))
            rows, cols = linear_sum_assignment(augmented)
            for row, col in zip(rows, cols):
                if col < len(points) and costs[row, col] < self.association_gate:
                    matched[int(row)] = int(col)
                    used.add(int(col))
        for i, track in enumerate(self.tracks):
            hit = i in matched
            track.history.append(hit)
            if hit:
                col = matched[i]
                track.update(points[col], col < pair_count)
        self.tracks = [t for t in self.tracks if sum(t.history) >= 12]
        existing_tracks = list(self.tracks)
        for i in range(pair_count):
            if i in used:
                continue
            if any(t.distance(points[i]) < self.association_gate for t in existing_tracks):
                continue
            x, y = points[i]
            newborn = Track(self.next_id, np.array([x, 0., y, 0.]))
            newborn.update(points[i], False)
            self.tracks.append(newborn)
            self.next_id += 1
        return self.tracks
