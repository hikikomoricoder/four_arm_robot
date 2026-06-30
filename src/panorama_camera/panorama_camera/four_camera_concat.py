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
    debug_concat = False  # True → show incremental panorama at debug_pair
    debug_pair = 1        # which pair to show: 0 (cam0↔1), 1 (cam1↔2), 2 (cam2↔3)
    blend_method = 2  # 0=baseline(weighted avg), 1=multiband, 2=seam, 3=exponent^3, 4=best-image
    force_center_alignment = True  # constrain image centres to same height (horizontal camera array)

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

        # Geometric sanity limits for a pairwise homography.  These keep the
        # panorama from exploding when feature matching produces a degenerate
        # perspective estimate.
        self._max_pair_scale = 2.0
        self._min_pair_scale = 0.5
        self._max_pair_perspective = 0.1
        self._max_pair_width_ratio = 1.5
        self._max_pair_height_ratio = 1.2

        # Hard upper bound on the final panorama size (width, height) relative
        # to the size of the first input image.
        self._max_canvas_width_ratio = 4.0
        self._max_canvas_height_ratio = 1.5

        self._log = logging.get_logger('four_camera_stitcher')

    def request_recompute(self):
        """Request that the stitching geometry be recomputed on the next frame."""
        self._recompute = True

    @property
    def ready(self):
        return self._ready

    def _pair_homography_is_reasonable(self, H, w, h):
        """Return True if H maps an adjacent image without crazy scaling.

        The panorama canvas is computed from the transformed image corners, so
        a degenerate perspective estimate (large scale/shear/perspective
        terms) makes the output explode.  This check rejects such estimates.
        """
        if H is None or H.shape != (3, 3):
            return False

        # Linear-part scale must be close to 1 (no zoom/flip explosion).
        A = H[:2, :2].astype(np.float64)
        s = np.linalg.svd(A, compute_uv=False)
        if np.any(s > self._max_pair_scale) or np.any(s < self._min_pair_scale):
            return False

        # Perspective terms must be tiny for a roughly planar camera array.
        perspective = abs(H[2, 0]) * w + abs(H[2, 1]) * h
        if perspective > self._max_pair_perspective:
            return False

        # Transformed image bounding box must stay close to original size.
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        warped_corners = cv2.perspectiveTransform(corners, H)
        xs = warped_corners[:, 0, 0]
        ys = warped_corners[:, 0, 1]
        if (xs.max() - xs.min()) > self._max_pair_width_ratio * w:
            return False
        if (ys.max() - ys.min()) > self._max_pair_height_ratio * h:
            return False
        return True

    def _constrain_center_height(self, H, w, h):
        """Adjust homography y-translation so the image centre maps to h/2.

        All rotation, scale, and perspective terms of ``H`` are preserved;
        only ``H[1, 2]`` (vertical translation) is recomputed so that the
        source image centre ``(w/2, h/2)`` maps to height ``h/2`` in the
        destination plane.  This enforces the constraint that adjacent
        cameras at the same height produce images whose centres align
        vertically, eliminating the cumulative vertical drift that creates
        black borders on the panorama.
        """
        cx, cy = w / 2.0, h / 2.0
        denom = H[2, 0] * cx + H[2, 1] * cy + H[2, 2]
        if abs(denom) < 1e-10:
            return H
        new_h12 = cy * denom - H[1, 0] * cx - H[1, 1] * cy
        H_out = H.copy()
        H_out[1, 2] = new_h12
        return H_out

    def _estimate_translation_homography(self, src_pts, dst_pts):
        """Fit a pure 2-D translation from src_pts to dst_pts.

        Returns (H, inlier_mask) or (None, None) if too few inliers.
        """
        if src_pts is None or dst_pts is None or len(src_pts) < self.min_matches:
            return None, None

        dx = float(np.median(dst_pts[:, 0, 0] - src_pts[:, 0, 0]))
        dy = float(np.median(dst_pts[:, 0, 1] - src_pts[:, 0, 1]))

        pred = src_pts + np.array([[[dx, dy]]], dtype=np.float32)
        errs = np.linalg.norm(pred - dst_pts, axis=2)
        inliers = (errs < self.ransac_thresh).reshape(-1, 1)
        n_inliers = int(np.sum(inliers))
        if n_inliers < self.min_matches:
            return None, None

        H = np.array([[1.0, 0.0, dx],
                      [0.0, 1.0, dy],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        return H, inliers

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

        # Reject degenerate perspective estimates that would make the panorama
        # canvas explode, and fall back to a pure translation model.  The
        # matches themselves are kept unchanged.
        if not self._pair_homography_is_reasonable(H, w, h):
            self._log.info(
                f'Full homography unreasonable ({strategy}); '
                f'falling back to translation model')
            H, mask = self._estimate_translation_homography(src_pts, dst_pts)
            n_inliers = int(np.sum(mask)) if mask is not None else 0
            if H is None or n_inliers < self.min_matches:
                self._log.info(
                    f'Translation fallback failed ({strategy}): '
                    f'{n_inliers} inliers < {self.min_matches}')
                if strategy == 'cross-check':
                    fail_dm = [cv2.DMatch(_queryIdx=r, _trainIdx=l, _distance=0)
                               for r, l in mutual]
                else:
                    fail_dm = good
                _debug_show(f'translation fail', fail_dm)
                return None
            strategy = f'{strategy}+translation'

        self._log.info(
            f'Homography OK ({strategy}): {len(src_pts)} matches, '
            f'{n_inliers} inliers')

        # Keep only the inliers used by the accepted model for debug drawing.
        inlier_indices = np.where(mask.ravel())[0]
        if strategy.startswith('cross-check'):
            ok_dm = [cv2.DMatch(_queryIdx=mutual[i][0],
                                _trainIdx=mutual[i][1],
                                _distance=0)
                     for i in inlier_indices]
        else:
            ok_dm = [good[i] for i in inlier_indices]
        _debug_show(f'{strategy} OK', ok_dm)

        # ---- centre-height post-correction ---------------------------------
        # Adjust the vertical translation of H so that the source image
        # centre maps to the same height as the destination centre.  This is
        # applied *after* estimation and outlier rejection, leaving the
        # matching process completely untouched.  It corrects the cumulative
        # vertical drift caused by depth parallax and spurious rotation
        # without sacrificing stitch quality.
        if self.force_center_alignment:
            H = self._constrain_center_height(H, w, h)
            self._log.info(
                f'Applied centre-height correction ({strategy})')

        return H

    def _debug_show_concat(self, images, cumulative, step):
        """Render and show the intermediate panorama up to ``step``.

        ``step`` 0 shows image 1 stitched onto image 0, step 1 adds image 2
        to the already-stitched image 0+1, and step 2 adds image 3.
        The window is only shown when ``step == self.debug_pair``.
        """
        if not self.debug_concat or step != self.debug_pair:
            return

        end = step + 2
        partial_images = images[:end]
        partial_H = cumulative[:end]

        h, w = images[0].shape[:2]
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        all_corners = np.concatenate(
            [cv2.perspectiveTransform(corners, H) for H in partial_H], axis=0)
        min_xy = np.floor(all_corners.min(axis=0).ravel()).astype(np.int32)
        max_xy = np.ceil(all_corners.max(axis=0).ravel()).astype(np.int32)

        canvas_w = max(max_xy[0] - min_xy[0], 1)
        canvas_h = max(max_xy[1] - min_xy[1], 1)
        canvas_size = (canvas_w, canvas_h)

        T = np.array([[1, 0, -min_xy[0]],
                      [0, 1, -min_xy[1]],
                      [0, 0, 1]], dtype=np.float64)
        warp_H = [T @ H for H in partial_H]

        # Clamp the debug canvas to the same limits used for the final output.
        max_canvas_w = int(self._max_canvas_width_ratio * w)
        max_canvas_h = int(self._max_canvas_height_ratio * h)
        if canvas_w > max_canvas_w or canvas_h > max_canvas_h:
            scale = min(max_canvas_w / canvas_w, max_canvas_h / canvas_h)
            S = np.array([[scale, 0, 0],
                          [0, scale, 0],
                          [0, 0, 1]], dtype=np.float64)
            warp_H = [S @ H for H in warp_H]
            canvas_w = int(canvas_w * scale)
            canvas_h = int(canvas_h * scale)
            canvas_size = (canvas_w, canvas_h)

        accumulator = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
        weights = np.zeros((canvas_h, canvas_w), dtype=np.float32)
        ones = np.ones((h, w), dtype=np.uint8)

        for img, H in zip(partial_images, warp_H):
            warped = cv2.warpPerspective(img, H, canvas_size)
            mask = cv2.warpPerspective(ones, H, canvas_size)
            dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5).astype(np.float32)
            accumulator += warped.astype(np.float32) * dist[:, :, None]
            weights += dist

        valid = weights > 0
        accumulator[valid] /= weights[valid, None]
        panorama = np.clip(accumulator, 0, 255).astype(np.uint8)

        win_name = f'concat step {step} ({step+1} images)'
        cv2.imshow(win_name, panorama)
        cv2.waitKey(1)

    # ── Blending helpers ──────────────────────────────────────────────────────

    def _weighted_blend(self, warped_images, weight_maps, exponent=1.0):
        """Weighted average with optional exponent on distance weights.

        ``exponent=1.0`` → standard feathering (baseline).
        ``exponent=3.0`` → sharper transition, less ghosting.
        """
        h, w = warped_images[0].shape[:2]
        accumulator = np.zeros((h, w, 3), dtype=np.float32)
        weights = np.zeros((h, w), dtype=np.float32)

        for warped, dist in zip(warped_images, weight_maps):
            w = dist if exponent == 1.0 else np.power(dist, exponent)
            accumulator += warped.astype(np.float32) * w[..., None]
            weights += w

        valid = weights > 0
        accumulator[valid] /= weights[valid, None]
        return np.clip(accumulator, 0, 255).astype(np.uint8)

    def _multiband_blend(self, warped_images, weight_maps, num_levels=4):
        """Multi-band (Laplacian pyramid) blending — gold standard for deghosting.

        Low frequencies are blended over a wide area; high frequencies (details)
        are blended over a narrow area, preserving sharpness while removing ghosts.
        """
        h, w = warped_images[0].shape[:2]
        n = len(warped_images)

        # Normalise weight maps so they sum to 1 per pixel.
        wsum = np.sum(weight_maps, axis=0) + 1e-10
        weight_maps = [w / wsum for w in weight_maps]

        # Ensure pyramid levels are valid for small canvases.
        max_levels = int(np.floor(np.log2(min(h, w))))
        num_levels = min(num_levels, max(max_levels, 1))

        result = np.zeros((h, w, 3), dtype=np.float32)

        for img, wmap in zip(warped_images, weight_maps):
            img_f = img.astype(np.float32)
            # Keep weight map as 2D; add trailing dim at multiplication time
            # to avoid cv2.pyrDown squeezing (h,w,1) back to (h,w).

            # Gaussian pyramid of the weight map.
            gp_w = [wmap]
            for _ in range(num_levels):
                gp_w.append(cv2.pyrDown(gp_w[-1]))

            # Laplacian pyramid of the image.
            current = img_f.copy()
            lp = []
            for _ in range(num_levels):
                down = cv2.pyrDown(current)
                up = cv2.pyrUp(down, dstsize=(current.shape[1], current.shape[0]))
                lp.append(current - up)   # high-frequency band
                current = down
            lp.append(current)            # base (lowest frequency)

            # Blend each pyramid level independently.
            blended = [lv * gw[..., None] for lv, gw in zip(lp, gp_w)]

            # Reconstruct from the blended pyramid.
            recon = blended[num_levels]
            for i in range(num_levels - 1, -1, -1):
                sz = (blended[i].shape[1], blended[i].shape[0])
                recon = cv2.pyrUp(recon, dstsize=sz) + blended[i]
            result += recon

        return np.clip(result, 0, 255).astype(np.uint8)

    def _seam_blend(self, warped_images, weight_maps, seam_radius=10):
        """Seam-guided narrow feathering.

        Find the per-pixel "best image" via argmax on distance weights, detect
        boundaries between image labels, and feather only within ``seam_radius``
        pixels of those boundaries.  Outside the transition zone the output is
        taken from a single image, eliminating ghosting.
        """
        h, w = warped_images[0].shape[:2]
        n = len(warped_images)

        # Per-pixel best-image label by argmax over distance weights.
        wstack = np.stack(weight_maps, axis=-1)            # (h, w, n)
        best_idx = np.argmax(wstack, axis=-1).astype(np.float32)

        # Detect boundaries where the best-image label changes.
        grad_x = np.abs(np.diff(best_idx, axis=1, append=best_idx[:, -1:])) > 0.5
        grad_y = np.abs(np.diff(best_idx, axis=0, append=best_idx[-1:, :])) > 0.5
        boundary = (grad_x | grad_y).astype(np.uint8) * 255

        # Distance from each pixel to the nearest boundary.
        dist = cv2.distanceTransform(255 - boundary, cv2.DIST_L2, 5).astype(np.float32)
        feather = np.clip(dist / max(seam_radius, 1), 0, 1)   # (h, w)

        result = np.zeros((h, w, 3), dtype=np.float32)
        alpha_sum = np.zeros((h, w, 1), dtype=np.float32)

        for idx in range(n):
            hard = (best_idx == idx).astype(np.float32)[..., None]               # (h, w, 1)
            smoothed = cv2.GaussianBlur(hard, (0, 0), seam_radius / 2.0)         # → (h, w), restore dim
            smoothed = smoothed[..., None]                                       # (h, w, 1)
            alpha = feather[..., None] * hard + (1.0 - feather[..., None]) * smoothed
            result += warped_images[idx].astype(np.float32) * alpha
            alpha_sum += alpha

        valid = alpha_sum.squeeze() > 0
        result[valid] /= alpha_sum[valid]
        return np.clip(result, 0, 255).astype(np.uint8)

    def _best_image_blend(self, warped_images, weight_maps, smooth_sigma=1.5):
        """Per-pixel best image selection with light Gaussian blur at seams.

        Each pixel is taken from whichever image has the largest distance weight.
        A light Gaussian blur is applied across the entire panorama to hide hard
        transitions.  Computationally the cheapest deghosting approach.
        """
        n = len(warped_images)
        wstack = np.stack(weight_maps, axis=-1)
        best = np.argmax(wstack, axis=-1).astype(np.uint8)

        # Vectorised gather: build result from best-image labels.
        h, w = warped_images[0].shape[:2]
        result = np.zeros((h, w, 3), dtype=np.uint8)
        for i in range(n):
            mask = (best == i)
            result[mask] = warped_images[i][mask]

        # Light blur to suppress hard seam edges.
        blurred = cv2.GaussianBlur(result.astype(np.float32), (0, 0), smooth_sigma)
        return np.clip(blurred, 0, 255).astype(np.uint8)

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

        # Cumulative homographies map every image onto the plane of image 0.
        cumulative = [np.eye(3, dtype=np.float64)]
        for i in range(3):
            H = self._match_pair(images[i], images[i + 1], pair_idx=i)
            self._log.info(f'Computed homography {i} -> {i+1}: {H}')
            if H is None:
                self._ready = False
                return False
            adj.append(H)
            cumulative.append(cumulative[-1] @ H)
            self._debug_show_concat(images, cumulative, i)

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

        warp_homographies = [T @ H for H in cumulative]

        # Final safety net: the stitched panorama must not exceed the
        # configured maximum size.  If it does, scale all warps uniformly so
        # the output stays viewable while preserving relative alignment.
        max_canvas_w = int(self._max_canvas_width_ratio * w)
        max_canvas_h = int(self._max_canvas_height_ratio * h)
        if canvas_w > max_canvas_w or canvas_h > max_canvas_h:
            scale = min(max_canvas_w / canvas_w, max_canvas_h / canvas_h)
            self._log.info(
                f'Canvas {canvas_w}x{canvas_h} exceeds limits '
                f'({max_canvas_w}x{max_canvas_h}); scaling by {scale:.3f}')
            S = np.array([[scale, 0, 0],
                          [0, scale, 0],
                          [0, 0, 1]], dtype=np.float64)
            warp_homographies = [S @ H for H in warp_homographies]
            canvas_w = int(canvas_w * scale)
            canvas_h = int(canvas_h * scale)

        self._adj_homographies = adj
        self._warp_homographies = warp_homographies
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

        # Warp all images to the canvas and compute distance weight maps.
        warped_images = []
        weight_maps = []
        ones = np.ones((h, w), dtype=np.uint8)

        for img, H in zip(images, self._warp_homographies):
            warped = cv2.warpPerspective(img, H, self._canvas_size)
            mask = cv2.warpPerspective(ones, H, self._canvas_size)
            dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5).astype(np.float32)
            warped_images.append(warped)
            weight_maps.append(dist)

        # Dispatch to the selected blending method.
        if self.blend_method == 1:
            panorama = self._multiband_blend(warped_images, weight_maps)
        elif self.blend_method == 2:
            panorama = self._seam_blend(warped_images, weight_maps, seam_radius=20)
        elif self.blend_method == 3:
            panorama = self._weighted_blend(warped_images, weight_maps, exponent=3.0)
        elif self.blend_method == 4:
            panorama = self._best_image_blend(warped_images, weight_maps, smooth_sigma=1.5)
        else:
            panorama = self._weighted_blend(warped_images, weight_maps, exponent=1.0)

        return panorama

    def get_status(self):
        """Return a short human-readable status string."""
        if not self._ready:
            return 'Not ready'
        if self._recompute:
            return 'Recompute requested'
        return 'Ready'
