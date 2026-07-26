"""
Stereo distance estimator using a pre-computed disparity map.

Usage:
    estimator = StereoDistanceEstimator(focal_length=381.3, baseline=0.08)
    estimator.update_disparity(disparity_msg)   # stereo_msgs/DisparityImage
    distance = estimator.estimate_distance(bbox) # bbox = (x1, y1, x2, y2)

Camera parameters are taken from the DisparityImage message by default,
or can be overridden at construction time.

For this project the stereo pair is camera_5 (left) / camera_6 (right):
    focal_length = 381.3 px   (horizontal_fov = 1.3962634 rad, width = 640)
    baseline     = 0.08 m     (camera_5 y = +0.04, camera_6 y = -0.04)
"""

import numpy as np
from cv_bridge import CvBridge


class StereoDistanceEstimator:
    """Estimates metric depth of detected objects using a stereo disparity map.

    The depth formula is:
        Z = (focal_length * baseline) / disparity

    where:
        focal_length  -- fx in pixels (horizontal focal length of the rectified left camera)
        baseline      -- distance between the two cameras in metres
        disparity     -- disparity value (pixels) at the object location
    """

    def __init__(self, focal_length: float | None = None, baseline: float | None = None):
        """
        Args:
            focal_length: fx in pixels.  When None, the value is read from
                          each incoming DisparityImage message (field ``f``).
            baseline:     Stereo baseline in metres.  When None, the absolute
                          value of the DisparityImage message field ``t`` is used.
        """
        self._focal_length_override = focal_length
        self._baseline_override = baseline

        self._bridge = CvBridge()

        # Latest disparity data (populated by update_disparity)
        self._disparity_array: np.ndarray | None = None
        self._msg_focal_length: float | None = None
        self._msg_baseline: float | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_disparity(self, disparity_msg) -> None:
        """Ingest a ``stereo_msgs/DisparityImage`` message.

        The disparity image is converted to a float32 NumPy array and stored
        internally so that ``estimate_distance`` can be called without passing
        it explicitly on every call.

        Args:
            disparity_msg: stereo_msgs.msg.DisparityImage
        """
        try:
            self._disparity_array = self._bridge.imgmsg_to_cv2(
                disparity_msg.image, desired_encoding="passthrough"
            ).astype(np.float32)
            # Cache the per-message intrinsics (used when no override was given)
            self._msg_focal_length = float(disparity_msg.f)
            self._msg_baseline = abs(float(disparity_msg.t))
        except Exception as exc:
            self._disparity_array = None
            raise RuntimeError(f"StereoDistanceEstimator: failed to decode disparity image – {exc}") from exc

    def estimate_distance(
        self,
        bbox: tuple[int, int, int, int],
        disparity_array: np.ndarray | None = None,
    ) -> float | None:
        """Estimate the metric distance to the object inside *bbox*.

        The median of all positive disparity values inside the bounding-box
        region is used for robustness against noise and partial occlusions.

        Args:
            bbox:             ``(x1, y1, x2, y2)`` pixel coordinates of the
                              detection bounding box (left-camera frame).
            disparity_array:  Optional float32 disparity map to use instead of
                              the internally cached one.

        Returns:
            Distance in metres, or ``None`` when estimation is not possible
            (no disparity data, no valid disparities in the region, etc.).
        """
        disp = disparity_array if disparity_array is not None else self._disparity_array
        if disp is None:
            return None

        focal_length = self._focal_length_override or self._msg_focal_length
        baseline = self._baseline_override or self._msg_baseline
        if focal_length is None or baseline is None or focal_length <= 0 or baseline <= 0:
            return None

        x1, y1, x2, y2 = (int(v) for v in bbox)
        h, w = disp.shape[:2]
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h))
        if x1 >= x2 or y1 >= y2:
            return None

        region = disp[y1:y2, x1:x2]
        valid = region[region > 0]
        if valid.size == 0:
            return None

        median_disp = float(np.median(valid))
        if median_disp <= 0:
            return None

        return (focal_length * baseline) / median_disp

    def has_disparity(self) -> bool:
        """Return True when a disparity map has been received."""
        return self._disparity_array is not None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def focal_length(self) -> float | None:
        return self._focal_length_override or self._msg_focal_length

    @property
    def baseline(self) -> float | None:
        return self._baseline_override or self._msg_baseline
