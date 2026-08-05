import math
from dataclasses import dataclass
from typing import List, Tuple

from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA


@dataclass
class Cluster:
    points: List[Tuple[float, float]]  # (x, y)


def hsv_to_rgba(h: float, s: float = 0.9, v: float = 0.9, a: float = 1.0) -> ColorRGBA:
    """Simple HSV->RGBA (0<=h<1)."""
    h = h % 1.0
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i = i % 6
    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def representative_point(points: List[Tuple[float, float]], cx: float, cy: float) -> Tuple[float, float]:
    """
    centroid (cx,cy)에 가장 가까운 '실제 포인트'를 대표점으로 선택.
    centroid가 공간에 뜨는 문제를 방지하기 위한 간단/강력한 방식.
    """
    return min(points, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)


def centroid(pts: List[Tuple[float, float]]) -> Tuple[float, float]:
    sx = 0.0
    sy = 0.0
    n = float(len(pts))
    for (x, y) in pts:
        sx += x
        sy += y
    return (sx / n, sy / n)


def scan_to_points(msg: LaserScan, max_range: float) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    angle = msg.angle_min

    rmin = max(msg.range_min, 0.0)
    rmax = min(msg.range_max, max_range)

    for r in msg.ranges:
        if math.isfinite(r) and (rmin <= r <= rmax):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            pts.append((x, y))
        angle += msg.angle_increment

    return pts


def filter_front_fov(
    points: List[Tuple[float, float]], front_angle_deg: float, fov_deg: float
) -> List[Tuple[float, float]]:
    """
    라이다 자체 좌표계 기준 각도가 front_angle_deg인 방향("실제 전방", 라이다가 뒤집혀
    장착된 경우 180도 등으로 보정된 값)을 중심으로 좌우 fov_deg/2씩만 남긴다.
    """
    front_rad = math.radians(front_angle_deg)
    half_fov = math.radians(fov_deg / 2.0)

    result: List[Tuple[float, float]] = []
    for (x, y) in points:
        angle = math.atan2(y, x)
        diff = math.atan2(math.sin(angle - front_rad), math.cos(angle - front_rad))
        if abs(diff) <= half_fov:
            result.append((x, y))
    return result


def euclidean_sequential_clustering(
    pts: List[Tuple[float, float]],
    cluster_tolerance: float,
    min_cluster_size: int,
    max_cluster_size: int,
) -> List[Cluster]:
    """
    2D LiDAR scan order 기반 간단 클러스터링.
    연속 포인트 간 거리 <= cluster_tolerance이면 같은 클러스터로 묶음.
    """
    if not pts:
        return []

    clusters: List[Cluster] = []
    current: List[Tuple[float, float]] = [pts[0]]

    def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx, dy)

    for i in range(1, len(pts)):
        if dist(pts[i - 1], pts[i]) <= cluster_tolerance:
            current.append(pts[i])
            if len(current) > max_cluster_size:
                clusters.append(Cluster(points=current))
                current = []
        else:
            if len(current) >= min_cluster_size:
                clusters.append(Cluster(points=current))
            current = [pts[i]]

    if len(current) >= min_cluster_size:
        clusters.append(Cluster(points=current))

    return clusters
