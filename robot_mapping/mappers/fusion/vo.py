"""Sparse RGB-D visual odometry: ORB features + depth -> PnP RANSAC (OpenCV only).

Frame-to-keyframe tracking: the reference keyframe carries 3-D points (from its
depth image, in its own camera optical frame); the current frame contributes 2-D
keypoints.  ``solvePnPRansac`` returns the pose of the reference points in the
current camera, i.e. ``T_cur_ref``.  The same routine is used for loop-closure
verification between two keyframes.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class Features:
    kp: np.ndarray            # (N, 2) float32 pixel coordinates (u, v)
    desc: np.ndarray          # (N, 32) uint8 ORB descriptors
    pts3d: np.ndarray         # (N, 3) float32 in the camera optical frame, NaN where depth is missing

    def __len__(self):
        return len(self.kp)


@dataclass
class PnPResult:
    ok: bool
    T_cur_ref: np.ndarray     # 4x4
    inliers: int
    matches: int
    inlier_idx: Optional[np.ndarray] = None   # indices into the match list


class FeatureTracker:
    def __init__(self, K: np.ndarray, n_features: int = 1000, ratio: float = 0.8,
                 min_inliers: int = 15, reproj_err: float = 3.0, min_depth_m: float = 0.25,
                 max_depth_m: float = 6.0):
        self.K = np.asarray(K, dtype=np.float64)
        self.orb = cv2.ORB_create(nfeatures=n_features, scaleFactor=1.2, nlevels=8, edgeThreshold=19,
                                  patchSize=31, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.ratio = ratio
        self.min_inliers = min_inliers
        self.reproj_err = reproj_err
        self.min_mm = int(min_depth_m * 1000)
        self.max_mm = int(max_depth_m * 1000)

    # ------------------------------------------------------------ features
    def extract(self, gray: np.ndarray, depth_mm: np.ndarray, drop_mask: Optional[np.ndarray] = None) -> Features:
        kps, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kps) == 0:
            return Features(np.zeros((0, 2), np.float32), np.zeros((0, 32), np.uint8), np.zeros((0, 3), np.float32))
        kp = np.array([k.pt for k in kps], dtype=np.float32)
        if drop_mask is not None:
            u = np.clip(kp[:, 0].astype(int), 0, drop_mask.shape[1] - 1)
            v = np.clip(kp[:, 1].astype(int), 0, drop_mask.shape[0] - 1)
            keep = ~drop_mask[v, u]
            kp, desc = kp[keep], desc[keep]
        pts3d = self._lift(kp, depth_mm)
        return Features(kp, desc, pts3d)

    def _lift(self, kp: np.ndarray, depth_mm: np.ndarray) -> np.ndarray:
        h, w = depth_mm.shape
        u = np.clip(np.round(kp[:, 0]).astype(int), 1, w - 2)
        v = np.clip(np.round(kp[:, 1]).astype(int), 1, h - 2)
        # 3x3 neighbourhood median of valid depths (robust to single missing pixels)
        stack = np.stack([depth_mm[v + dv, u + du] for dv in (-1, 0, 1) for du in (-1, 0, 1)], axis=1).astype(np.float32)
        stack[(stack < self.min_mm) | (stack > self.max_mm)] = np.nan
        with np.errstate(all="ignore"):
            d = np.nanmedian(stack, axis=1) * 1e-3
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        x = (kp[:, 0] - cx) / fx * d
        y = (kp[:, 1] - cy) / fy * d
        return np.stack([x, y, d], axis=1).astype(np.float32)

    # ------------------------------------------------------------ matching
    def match(self, desc_ref: np.ndarray, desc_cur: np.ndarray) -> np.ndarray:
        """Ratio-test matches -> (M, 2) array of (ref_idx, cur_idx)."""
        if len(desc_ref) < 2 or len(desc_cur) < 2:
            return np.zeros((0, 2), dtype=int)
        knn = self.matcher.knnMatch(desc_ref, desc_cur, k=2)
        out = []
        for m in knn:
            if len(m) == 2 and m[0].distance < self.ratio * m[1].distance:
                out.append((m[0].queryIdx, m[0].trainIdx))
            elif len(m) == 1:
                out.append((m[0].queryIdx, m[0].trainIdx))
        return np.array(out, dtype=int).reshape(-1, 2)

    # ------------------------------------------------------------ PnP
    def estimate(self, ref: Features, cur: Features, T_guess_cur_ref: Optional[np.ndarray] = None,
                 min_inliers: Optional[int] = None) -> PnPResult:
        """Pose of ``ref`` points seen from ``cur``: x_cur = R x_ref + t."""
        min_inl = self.min_inliers if min_inliers is None else min_inliers
        fail = PnPResult(False, np.eye(4) if T_guess_cur_ref is None else T_guess_cur_ref.copy(), 0, 0)
        m = self.match(ref.desc, cur.desc)
        if len(m) < 6:
            fail.matches = len(m)
            return fail
        obj = ref.pts3d[m[:, 0]]
        img = cur.kp[m[:, 1]]
        valid = np.isfinite(obj).all(axis=1)
        obj, img, m = obj[valid], img[valid], m[valid]
        if len(obj) < 6:
            fail.matches = len(m)
            return fail
        obj = obj.astype(np.float64).reshape(-1, 1, 3)
        img = img.astype(np.float64).reshape(-1, 1, 2)
        if T_guess_cur_ref is not None:
            rvec, _ = cv2.Rodrigues(T_guess_cur_ref[:3, :3])
            tvec = T_guess_cur_ref[:3, 3].reshape(3, 1).copy()
            use_guess = True
        else:
            rvec = np.zeros((3, 1)); tvec = np.zeros((3, 1)); use_guess = False
        try:
            ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, self.K, None, rvec, tvec, useExtrinsicGuess=use_guess,
                                                     iterationsCount=150, reprojectionError=self.reproj_err,
                                                     confidence=0.995, flags=cv2.SOLVEPNP_EPNP)
        except cv2.error:
            fail.matches = len(m)
            return fail
        if not ok or inl is None or len(inl) < min_inl:
            fail.matches = len(m)
            fail.inliers = 0 if inl is None else len(inl)
            return fail
        inl = inl.reshape(-1)
        try:
            rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], self.K, None, rvec, tvec)
        except cv2.error:
            pass
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)
        # sanity: reject wild jumps (bad RANSAC consensus)
        if not np.isfinite(T).all() or np.linalg.norm(T[:3, 3]) > 5.0:
            fail.matches = len(m)
            return fail
        return PnPResult(True, T, int(len(inl)), int(len(m)), m[inl])
