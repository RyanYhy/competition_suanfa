#!/usr/bin/env python3
"""Image-only, closed-set organizer-template recognition (not general-purpose OCR)."""
import json
from pathlib import Path
import cv2
import numpy as np


class TemplateRecognizer:
    def __init__(self, root):
        self.root = Path(root)
        self.orb = cv2.ORB_create(nfeatures=5000, scaleFactor=1.15, nlevels=12,
                                  edgeThreshold=8, patchSize=21, fastThreshold=8)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.templates = []
        self.masks = {}
        records = json.loads((self.root/'config/recognition_assets.json').read_text())
        for color in ('red', 'yellow', 'green'):
            for state in ('on', 'off'):
                records.append(dict(id='signal_'+color+'_'+state, category='signal',
                                    label=color+'_'+state, color=color, state=state,
                                    texture='materials/textures/signal_'+color+'_'+state+'.png'))
        for record in records:
            image = cv2.imread(str(self.root/record['texture']), cv2.IMREAD_UNCHANGED)
            if image is None:
                raise ValueError('Missing recognition template: '+record['texture'])
            mask = np.full(image.shape[:2], 255, dtype=np.uint8)
            if image.ndim == 3 and image.shape[2] == 4:
                mask = np.where(image[:, :, 3] > 200, 255, 0).astype(np.uint8)
                alpha = image[:, :, 3:4].astype(float)/255
                image = (image[:, :, :3]*alpha+255*(1-alpha)).astype(np.uint8)
            if record['category'] == 'plate':
                image = cv2.resize(image, (253, 80))  # Match official 9.5 : 3 physical aspect.
            elif record['category'] == 'signal':
                image = cv2.resize(image, (160, 160))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            mask = cv2.resize(mask, (gray.shape[1], gray.shape[0]), interpolation=cv2.INTER_NEAREST)
            self.masks[record['id']] = mask > 0
            keypoints, descriptors = self.orb.detectAndCompute(gray, cv2.erode(mask, np.ones((9, 9), np.uint8)))
            self.templates.append((record, gray, keypoints, descriptors))

    def detect(self, image, signals=False):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        points, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None:
            return []
        result = []
        for record, template, keys, desc in self.templates:
            if desc is None or (record['category'] == 'signal' and not signals):
                continue
            pairs = self.matcher.knnMatch(desc, descriptors, k=2)
            matches = [pair[0] for pair in pairs if len(pair) == 2 and pair[0].distance < .72*pair[1].distance]
            # A single image feature cannot substantiate multiple template features.
            matches = sorted(matches, key=lambda m: m.distance)
            unique = {}; matches = [unique.setdefault(m.trainIdx, m) for m in matches if m.trainIdx not in unique]
            if len(matches) < 8:
                continue
            src = np.float32([keys[m.queryIdx].pt for m in matches])
            dst = np.float32([points[m.trainIdx].pt for m in matches])
            homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if homography is None or mask is None:
                continue
            valid = mask.ravel().astype(bool)
            if valid.sum() < 8 or valid.mean() < .60:
                continue
            h, w = template.shape
            corners = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
            quad = cv2.perspectiveTransform(corners[None], homography)[0]
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
                continue
            area = cv2.contourArea(quad, oriented=True)
            if area < 180 or area > gray.size*.75:
                continue
            # Inliers must cover the target, not just a shared word or a single corner.
            coverage = cv2.contourArea(cv2.convexHull(src[valid]))/(w*h)
            if coverage < .10:
                continue
            x1, y1 = np.floor(quad.min(axis=0)).astype(int)
            x2, y2 = np.ceil(quad.max(axis=0)).astype(int)
            if x1 < 0 or y1 < 0 or x2 >= gray.shape[1] or y2 >= gray.shape[0]:
                continue
            if record['category'] == 'person' and y2-y1 < gray.shape[0]*.14:
                continue  # Nearby observation only; avoid counting the opposite district in the background.
            try:
                rectified = cv2.warpPerspective(gray, np.linalg.inv(homography), (w, h))
            except np.linalg.LinAlgError:
                continue
            # Validate appearance independently of feature matches. Score is similarity,
            # not a calibrated probability. Unknown targets are left unlabelled.
            mask_pixels = self.masks[record['id']]
            observed = rectified[mask_pixels].astype(float); expected = template[mask_pixels].astype(float)
            observed -= observed.mean(); expected -= expected.mean()
            score = float(observed.dot(expected)/max(1e-6, np.linalg.norm(observed)*np.linalg.norm(expected)))
            if not np.isfinite(score) or score < .60:
                continue
            detection = dict(record)
            detection.update(confidence=round(max(0., min(1., score)), 3), box=[int(x1), int(y1), int(x2), int(y2)])
            result.append(detection)
        # Suppress competing identities on the same visual target.
        selected = []
        for det in sorted(result, key=lambda r: r['confidence'], reverse=True):
            if any(self.overlap(det['box'], prior['box']) > .45 for prior in selected):
                continue
            selected.append(det)
        return selected

    @staticmethod
    def overlap(a, b):
        intersection = max(0, min(a[2], b[2])-max(a[0], b[0]))*max(0, min(a[3], b[3])-max(a[1], b[1]))
        union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
        return intersection/max(1, union)


def signal_state(detections):
    # Do not infer green from an absent red. Conflicting lamps are unknown.
    active = {d['color'] for d in detections if d['category'] == 'signal' and d['state'] == 'on'}
    return next(iter(active)) if len(active) == 1 else 'unknown'
