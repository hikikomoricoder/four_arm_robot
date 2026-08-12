"""Panorama information extraction utilities.

Provides helpers to enrich object detection results with azimuth
attributes derived from the panoramic X-to-angle mapping table
(``_last_intervals`` — 10° step boundaries from
:meth:`FourCameraStitcher.get_interval_boundaries`).
"""

from typing import List, Dict, Optional, Tuple
from collections import OrderedDict

from panorama_camera.coco_names_zh import COCO_NAMES_ZH, get_class_name_zh  # noqa: F401

# ---------------------------------------------------------------------------
# 8-direction sector definitions (45° each, centred on the cardinal / inter-
# cardinal directions).  The first sector ("前方") wraps across 0°.
# ---------------------------------------------------------------------------
_DIR_SECTORS: List[Tuple[str, Tuple[float, float]]] = [
    ("前方",   (-22.5,  22.5)),
    ("左前方", ( 22.5,  67.5)),
    ("左侧",   ( 67.5, 112.5)),
    ("左后方", (112.5, 157.5)),
    ("后方",   (157.5, 202.5)),
    ("右后方", (202.5, 247.5)),
    ("右侧",   (247.5, 292.5)),
    ("右前方", (-67.5,  -22.5)),
]


def _azimuth_in_sector(az: float, lo: float, hi: float) -> bool:
    """Check whether *az* (0–360) falls inside [*lo*, *hi*).

    Supports negative *lo* / *hi* to express ranges that cross the
    0° boundary (e.g. 前方: -22.5° ~ 22.5°; 右前方: -67.5° ~ -22.5°).
    """
    # Normalise both bounds to 0–360.
    lo_n = lo % 360.0
    hi_n = hi % 360.0
    if lo_n < hi_n:
        return lo_n <= az < hi_n
    else:
        # Wrap-around: range crosses 0° (e.g. 前方: lo_n=337.5, hi_n=22.5).
        return az >= lo_n or az < hi_n


def _interpolate_angle(
    x_pano: float,
    intervals: List[Tuple[float, Optional[List[Tuple[int, float]]]]],
) -> Optional[float]:
    """Map a panorama X coordinate to an azimuth angle via linear interpolation.

    Builds a sorted ``(X, theta)`` lookup from the primary hits of
    *intervals*, then linearly interpolates *x_pano* between the two
    nearest bracketing points.  When *x_pano* lies outside the mapped
    range the nearest boundary angle is returned.

    Args:
        x_pano: X coordinate in the **original** panorama canvas.
        intervals: ``_last_intervals`` — list of ``(theta_deg, hits)``
            where ``hits[0] == (cam_idx, X)``.

    Returns:
        Azimuth angle in degrees (normalised to [0, 360)), or
        ``None`` when the interval table is insufficient.
    """
    if not intervals:
        return None

    # Collect (X, theta) from primary hits; skip uncovered azimuths.
    points = []
    for theta, hits in intervals:
        if hits and len(hits) > 0:
            points.append((hits[0][1], theta))

    if len(points) < 2:
        return None

    # Sort by ascending panorama X so we can binary-search.
    points.sort(key=lambda p: p[0])
    xs = [p[0] for p in points]
    n = len(xs)

    # Clamp to the covered range.
    if x_pano <= xs[0]:
        return points[0][1] % 360.0
    if x_pano >= xs[-1]:
        return points[-1][1] % 360.0

    # Bisect to find the bracketing segment.
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x_pano:
            lo = mid
        else:
            hi = mid

    x0, th0 = points[lo]
    x1, th1 = points[hi]

    if abs(x1 - x0) < 1e-9:
        return th0 % 360.0

    frac = (x_pano - x0) / (x1 - x0)
    return (th0 + frac * (th1 - th0)) % 360.0


def enrich_detections_with_azimuth(
    detections: List[Dict],
    intervals: List[Tuple[float, Optional[List[Tuple[int, float]]]]],
    pano_w: int,
    display_w: int,
) -> List[Dict]:
    """Attach azimuth attributes to every detection from a panorama.

    Each detection's bbox centre X is mapped to an **azimuth angle**
    (the camera-ring azimuth of the object centre), and the bbox
    horizontal extent ``[x1, x2]`` is mapped to an **azimuth range**
    (the angular interval the object spans in the panorama).

    Both values are rounded to **integer degrees**.  The proportional
    conversion uses linear interpolation between the 10°-spaced
    boundaries provided by *intervals*.

    Detection bboxes are assumed to be in **display** coordinates
    (scaled to ``display_w`` pixels wide), while *intervals* store X
    in the original panorama canvas coordinates (``pano_w`` wide).
    The two spaces are reconciled via ``scale = pano_w / display_w``.

    Each returned dict is a shallow copy of the corresponding input
    dict with two additional keys:

    * ``azimuth_deg`` (:class:`int` or ``None``) — centre azimuth.
    * ``azimuth_range`` (``(int, int)`` or ``None``) — left-edge and
      right-edge azimuths defining the angular interval spanned by
      the bounding box.

    When the azimuth cannot be determined for a detection (e.g. the
    interval table is ``None``), the new keys are set to ``None``.

    Args:
        detections: List of detection dicts, each minimally containing
            ``'bbox': [x1, y1, x2, y2]``.
        intervals: ``_last_intervals`` from
            :meth:`FourCameraStitcher.get_interval_boundaries`.
        pano_w: Width of the original (full-resolution) panorama
            canvas in pixels.
        display_w: Width of the display-scaled panorama on which
            detection was run, in pixels.

    Returns:
        A new list of enriched detection dicts.
    """
    if intervals is None:
        intervals = []

    scale = pano_w / display_w if display_w > 0 else 1.0
    enriched = []

    for det in detections:
        bbox = det['bbox']
        x1, _, x2, _ = bbox

        # Convert display-coordinate X values to panorama-canvas X.
        cx_pano = (x1 + x2) / 2.0 * scale
        xl_pano = x1 * scale
        xr_pano = x2 * scale

        az_center = _interpolate_angle(cx_pano, intervals)
        az_left = _interpolate_angle(xl_pano, intervals)
        az_right = _interpolate_angle(xr_pano, intervals)

        entry = dict(det)
        entry['azimuth_deg'] = round(az_center) if az_center is not None else None

        if az_left is not None and az_right is not None:
            entry['azimuth_range'] = (round(az_left), round(az_right))
        else:
            entry['azimuth_range'] = None

        # Enrich with Chinese class name when available
        entry['class_name_zh'] = get_class_name_zh(det.get('class_name', ''))

        enriched.append(entry)

    return enriched


def build_azimuth_description(
    detections: List[Dict],
    use_zh: bool = True,
) -> str:
    """Build a human-readable directional summary of panoramic detections.

    Groups detections by their ``azimuth_deg`` attribute into 8
    cardinal/inter-cardinal sectors (45° each), then produces a
    Chinese natural-language string listing, per sector, the total
    object count and a per-class breakdown.

    The eight sectors and their degree ranges are:

    ===========  ==========
    Direction    Range
    ===========  ==========
    前方         -22.5° ~  22.5°
    左前方        22.5° ~  67.5°
    左侧          67.5° ~ 112.5°
    左后方       112.5° ~ 157.5°
    后方         157.5° ~ 202.5°
    右后方       202.5° ~ 247.5°
    右侧         247.5° ~ 292.5°
    右前方       292.5° ~ 337.5°
    ===========  ==========

    Example output::

        "前方有3个目标，椅子数量为2，人数量为1；左前方有1个目标，桌子数量为1"

    Args:
        detections: List of enriched detection dicts (must contain
            ``azimuth_deg`` and either ``class_name`` or
            ``class_name_zh``).
        use_zh: When ``True`` (default), prefer ``class_name_zh``;
            otherwise fall back to ``class_name``.

    Returns:
        Formatted description string.  Empty string when no
        detections have valid azimuth data.
    """
    # ── group detections by direction sector ──
    # OrderedDict preserves the canonical sector order.
    sectors: Dict[str, List[Dict]] = OrderedDict()
    for name, _ in _DIR_SECTORS:
        sectors[name] = []

    for det in detections:
        az = det.get('azimuth_deg')
        if az is None:
            continue
        for name, (lo, hi) in _DIR_SECTORS:
            if _azimuth_in_sector(float(az), lo, hi):
                sectors[name].append(det)
                break

    # ── build the description string ──
    parts: List[str] = []
    for dir_name, dir_dets in sectors.items():
        if not dir_dets:
            continue

        total = len(dir_dets)

        # Count per class.
        class_counts: Dict[str, int] = OrderedDict()
        for d in dir_dets:
            cls = (d.get('class_name_zh') if use_zh else None) or d.get('class_name', 'unknown')
            class_counts[cls] = class_counts.get(cls, 0) + 1

        breakdown = '，'.join(
            f'{cls}数量为{count}' for cls, count in class_counts.items()
        )
        parts.append(f'{dir_name}有{total}个目标，{breakdown}')

    return '；'.join(parts)
