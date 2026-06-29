import cv2
import numpy as np
from rclpy import logging

class FourCameraStitcher:
    """Stitch four horizontally arranged images using ORB feature matching.

    The stitching geometry is computed once and reused for subsequent frames.
    Call ``request_recompute()`` (or the ROS service created by the node) to
    force a recomputation of the stitching homographies.
    """

    # ---- debug knobs (edit directly in code, not via ROS params) -------
    debug_match = True   # True → show feature-match visualization
    debug_pair = 1        # which pair to show: 0 (cam0↔1), 1 (cam1↔2), 2 (cam2↔3)

    def __init__(self, nfeatures=600, match_ratio=0.95, ransac_thresh=8.0,
                 min_matches=6, crop_ratio=0.35, epipolar_thresh=50.0,
                 fast_threshold=20, grid_rows=6, grid_cols=6):
        self.nfeatures = nfeatures
        self.match_ratio = match_ratio
        self.ransac_thresh = ransac_thresh
        self.min_matches = min_matches
        self.crop_ratio = crop_ratio
        self.epipolar_thresh = epipolar_thresh
        self.fast_threshold = fast_threshold
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols

        self.orb = cv2.ORB_create(nfeatures=nfeatures,
                                  fastThreshold=fast_threshold)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)

        self._adj_homographies = None
        self._warp_homographies = None
        self._canvas_size = None
        self._ready = False
        self._recompute = True

        self._log = logging.get_logger('four_camera_stitcher')

    def request_recompute(self):
        """Request that the stitching geometry be recomputed on the next frame."""
        self._recompute = True

    @property
    def ready(self):
        return self._ready

    def _detect(self, img):
        """Return keypoints and descriptors for a BGR or grayscale image.

        The image is divided into a grid (``grid_rows`` × ``grid_cols``) and
        features are selected uniformly across grid cells to suppress local
        over-density.  ORB detection is performed on the full image first so
        that the detector always sees enough pixels for robust scale-space
        analysis, regardless of grid resolution.

        When both ``grid_rows`` and ``grid_cols`` are ≤ 1 the grid is
        effectively disabled and full-image detection is used directly.
        """
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        h, w = gray.shape[:2]

        # Fall back to full-image detection when grid is effectively disabled.
        if self.grid_rows <= 1 and self.grid_cols <= 1:
            return self.orb.detectAndCompute(gray, None)

        # --- Phase 1: detect on the full image (robust scale pyramid) ---
        kps, desc = self.orb.detectAndCompute(gray, None)
        if kps is None or len(kps) == 0:
            return [], None

        # --- Phase 2: enforce per-cell budget for uniform distribution ---
        # Cap the effective grid so cells are at least ~30 px.
        effective_rows = min(self.grid_rows, max(1, h // 30))
        effective_cols = min(self.grid_cols, max(1, w // 30))

        cell_h = h // effective_rows
        cell_w = w // effective_cols
        n_cells = effective_rows * effective_cols

        # Assign each keypoint to its containing grid cell.
        cell_indices = [[] for _ in range(n_cells)]
        for i, kp in enumerate(kps):
            x, y = kp.pt
            c = min(int(x // cell_w), effective_cols - 1)
            r = min(int(y // cell_h), effective_rows - 1)
            cell_indices[r * effective_cols + c].append(i)

        # Distribute total budget evenly across cells.
        base_per_cell = self.nfeatures // n_cells
        extra = self.nfeatures % n_cells

        # Allocate extra slots to cells that have the most candidates.
        cell_stats = sorted([(idx, len(idx_list))
                             for idx, idx_list in enumerate(cell_indices)],
                            key=lambda x: x[1], reverse=True)

        selected = []
        for cell_idx, count in cell_stats:
            budget = base_per_cell + (1 if extra > 0 else 0)
            extra -= 1
            # Within each cell keep the highest-response features.
            cell_list = cell_indices[cell_idx]
            cell_list.sort(key=lambda i_: kps[i_].response, reverse=True)
            selected.extend(cell_list[:budget])

        selected.sort()
        result_kps = [kps[i] for i in selected]
        result_desc = desc[selected] if desc is not None else None
        return result_kps, result_desc

    def _match_pair(self, left_img, right_img, pair_idx=-1):
        """Estimate the homography that maps right_img onto left_img's plane.

        Only a portion (``crop_ratio``) of each image is used for feature
        detection to suppress false matches from non-overlapping regions.
        Keypoints are offset back to full-image coordinates before homography
        estimation.

        Matching uses cross-check (mutual nearest-neighbour) first; falls back
        to unidirectional ratio-test matching if cross-check yields too few
        matches.  Matches are further filtered by an epipolar prior (vertical
        disparity threshold) that assumes roughly horizontal camera arrangement
        — this prior is valid in the baseline configuration and should be
        re-evaluated (via ``request_recompute``) when camera poses change.

        If ``debug_match`` is True and ``pair_idx == debug_pair``, the matched
        keypoints are drawn and shown in an OpenCV window.
        """
        h, w = left_img.shape[:2]
        crop_w = int(w * self.crop_ratio)

        # ---- crop to overlap regions (crop_ratio from each image) -----------
        left_crop = left_img[:, :crop_w]            # left portion of left image
        right_crop = right_img[:, w - crop_w:]      # right portion of right image

        kp_l_crop, des_l = self._detect(left_crop)
        kp_r_crop, des_r = self._detect(right_crop)

        # offset right keypoints back to full-image coordinates (do this early
        # so debug drawing always uses full-image coords)
        for kp in kp_r_crop:
            kp.pt = (kp.pt[0] + (w - crop_w), kp.pt[1])
        kp_l = kp_l_crop
        kp_r = kp_r_crop

        # ---- debug helper (captures kp_l, kp_r, pair_idx from outer scope) ---
        def _debug_show(status, dmatches=None):
            if not self.debug_match or pair_idx != self.debug_pair:
                return
            dm = dmatches if dmatches else []
            debug_img = cv2.drawMatches(
                right_img, kp_r, left_img, kp_l, dm, None,
                matchColor=(0, 255, 0), singlePointColor=(0, 255, 0))
            win_name = f'{status} Pair {pair_idx} '
            cv2.imshow(win_name, debug_img)
            cv2.waitKey(1)

        if des_l is None or des_r is None:
            _debug_show('no descriptors')
            return None

        # ---- helper: unidirectional ratio-test (right → left) --------------
        def _unidirectional_match():
            matches = self.bf.knnMatch(des_r, des_l, k=2)
            good = []
            for pair in matches:
                if len(pair) != 2:
                    continue
                m, n = pair
                if m.distance < self.match_ratio * n.distance:
                    good.append(m)
            return good

        # ---- helper: cross-check (mutual best match) -----------------------
        def _cross_check_match():
            # right → left
            fwd_raw = self.bf.knnMatch(des_r, des_l, k=2)
            fwd = {}
            for pair in fwd_raw:
                if len(pair) != 2:
                    continue
                m, n = pair
                if m.distance < self.match_ratio * n.distance:
                    fwd[m.queryIdx] = m.trainIdx  # r_idx → l_idx

            # left → right
            bwd_raw = self.bf.knnMatch(des_l, des_r, k=2)
            bwd = {}
            for pair in bwd_raw:
                if len(pair) != 2:
                    continue
                m, n = pair
                if m.distance < self.match_ratio * n.distance:
                    bwd[m.queryIdx] = m.trainIdx  # l_idx → r_idx

            # keep only mutual matches
            mutual = []
            for r_idx, l_idx in fwd.items():
                if l_idx in bwd and bwd[l_idx] == r_idx:
                    mutual.append((r_idx, l_idx))
            return mutual

        # ---- try cross-check first, fall back to unidirectional -------------
        strategy = 'cross-check'
        mutual = _cross_check_match()
        if len(mutual) >= self.min_matches:
            src_pts = np.float32(
                [kp_r[r_idx].pt for r_idx, _ in mutual]).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [kp_l[l_idx].pt for _, l_idx in mutual]).reshape(-1, 1, 2)
        else:
            strategy = 'unidirectional'
            self._log.info(
                f'Cross-check gave {len(mutual)} matches (< {self.min_matches}), '
                f'falling back to unidirectional matching')
            good = _unidirectional_match()
            if len(good) < self.min_matches:
                self._log.info(
                    f'Unidirectional matches insufficient: '
                    f'{len(good)} < {self.min_matches}')
                _debug_show(f'uni fail', good)
                return None
            src_pts = np.float32(
                [kp_r[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [kp_l[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # ---- epipolar prior filter ------------------------------------------
        # Horizontal camera arrangement → epipolar lines ≈ horizontal
        # → corresponding points should share similar y-coordinates.
        y_diff = np.abs(src_pts[:, 0, 1] - dst_pts[:, 0, 1])
        epi_mask = (y_diff < self.epipolar_thresh).ravel()
        n_before = len(src_pts)
        src_pts = src_pts[epi_mask]
        dst_pts = dst_pts[epi_mask]
        n_after = len(src_pts)

        if n_after < n_before:
            self._log.info(
                f'Epipolar filter ({strategy}): {n_before} → {n_after} '
                f'({n_before - n_after} removed, '
                f'thresh={self.epipolar_thresh:.1f}px)')

        if n_after < self.min_matches:
            self._log.info(
                f'Insufficient matches after epipolar filter: '
                f'{n_after} < {self.min_matches}')
            # show pre-filter matches for diagnosis
            if strategy == 'cross-check':
                pre_dm = [cv2.DMatch(_queryIdx=r, _trainIdx=l, _distance=0)
                          for r, l in mutual]
            else:
                pre_dm = good
            _debug_show(f'epi fail')
            return None

        # also filter match-index lists for downstream use
        if strategy == 'cross-check':
            mutual = [(r, l) for (r, l), k in zip(mutual, epi_mask) if k]
        else:
            good = [m for m, k in zip(good, epi_mask) if k]

        # ---- homography estimation ------------------------------------------
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,
                                     self.ransac_thresh)
        n_inliers = int(np.sum(mask)) if mask is not None else 0
        if H is None or n_inliers < self.min_matches:
            self._log.info(
                f'Homography failed ({strategy}): '
                f'{n_inliers} inliers < {self.min_matches}')
            if strategy == 'cross-check':
                fail_dm = [cv2.DMatch(_queryIdx=r, _trainIdx=l, _distance=0)
                           for r, l in mutual]
            else:
                fail_dm = good
            _debug_show(f'H fail inliers', fail_dm)
            return None

        self._log.info(
            f'Homography OK ({strategy}): {len(src_pts)} matches, '
            f'{n_inliers} inliers')

        # ---- debug: draw final matches for the selected pair ----------------
        if strategy == 'cross-check':
            ok_dm = [cv2.DMatch(_queryIdx=r, _trainIdx=l, _distance=0)
                     for r, l in mutual]
        else:
            ok_dm = good
        _debug_show(f'{strategy} OK', ok_dm)

        return H

    def _normalize_images(self, images):
        """Return a list of images resized to the size of the first image."""
        h, w = images[0].shape[:2]
        normalized = []
        for img in images:
            if img.shape[:2] != (h, w):
                normalized.append(cv2.resize(img, (w, h)))
            else:
                normalized.append(img)
        return normalized

    def compute_stitch(self, images):
        """Compute the stitching geometry from four input images.

        Returns ``True`` if the geometry was successfully computed.
        """
        if len(images) != 4 or any(img is None for img in images):
            return False

        images = self._normalize_images(images)
        h, w = images[0].shape[:2]

        # Adjacent homographies: H_i maps image i+1 onto image i's plane.
        adj = []
        for i in range(3):
            H = self._match_pair(images[i], images[i + 1], pair_idx=i)
            self._log.info(f'Computed homography {i} -> {i+1}: {H}')
            if H is None:
                self._ready = False
                return False
            adj.append(H)

        # Cumulative homographies map every image onto the plane of image 0.
        cumulative = [np.eye(3, dtype=np.float64)]
        for H in adj:
            cumulative.append(cumulative[-1] @ H)

        # Compute the bounding box of all transformed images.
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        all_corners = np.concatenate(
            [cv2.perspectiveTransform(corners, H) for H in cumulative], axis=0)
        min_xy = np.floor(all_corners.min(axis=0).ravel()).astype(np.int32)
        max_xy = np.ceil(all_corners.max(axis=0).ravel()).astype(np.int32)

        canvas_w = max(max_xy[0] - min_xy[0], 1)
        canvas_h = max(max_xy[1] - min_xy[1], 1)

        # Shift so that the panorama origin is at (0, 0).
        T = np.array([[1, 0, -min_xy[0]],
                      [0, 1, -min_xy[1]],
                      [0, 0, 1]], dtype=np.float64)

        self._adj_homographies = adj
        self._warp_homographies = [T @ H for H in cumulative]
        self._canvas_size = (canvas_w, canvas_h)
        self._ready = True
        self._recompute = False
        return True

    def stitch(self, images):
        """Return the stitched panorama as a BGR image.

        The first call (or the first call after ``request_recompute``) computes
        the stitching geometry; afterwards the cached geometry is reused.
        Returns ``None`` if the geometry cannot be computed or images are missing.
        """
        if len(images) != 4 or any(img is None for img in images):
            return None

        if self._recompute or not self._ready or self._warp_homographies is None:
            if not self.compute_stitch(images):
                return None

        images = self._normalize_images(images)
        h, w = images[0].shape[:2]
        canvas_h, canvas_w = self._canvas_size[1], self._canvas_size[0]

        accumulator = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
        weights = np.zeros((canvas_h, canvas_w), dtype=np.float32)
        ones = np.ones((h, w), dtype=np.uint8)

        for img, H in zip(images, self._warp_homographies):
            warped = cv2.warpPerspective(img, H, self._canvas_size)
            mask = cv2.warpPerspective(ones, H, self._canvas_size)

            # Feather blending based on distance to the warped image border.
            dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5).astype(np.float32)
            accumulator += warped.astype(np.float32) * dist[:, :, None]
            weights += dist

        valid = weights > 0
        accumulator[valid] /= weights[valid, None]
        panorama = np.clip(accumulator, 0, 255).astype(np.uint8)
        return panorama

    def get_status(self):
        """Return a short human-readable status string."""
        if not self._ready:
            return 'Not ready'
        if self._recompute:
            return 'Recompute requested'
        return 'Ready'
