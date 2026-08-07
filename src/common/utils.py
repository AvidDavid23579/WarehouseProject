import math
from collections import defaultdict

import numpy as np
from numpy.typing import NDArray

from common.types import Pose


# Wraps angles to the interval [-pi, pi]
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))


def rotated_rectangle_vertices(
    pose: Pose, length: float, width: float
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    hl = length * 0.5
    hw = width * 0.5

    c = math.cos(pose.theta)
    s = math.sin(pose.theta)

    x = pose.x
    y = pose.y

    return (
        (x + hl * c - hw * s, y + hl * s + hw * c),
        (x + hl * c + hw * s, y + hl * s - hw * c),
        (x - hl * c + hw * s, y - hl * s - hw * c),
        (x - hl * c - hw * s, y - hl * s + hw * c),
    )


def update_rotated_rectangle_vertices(pose: NDArray[np.float32], vertices: NDArray[np.float32], length: float, width: float):
    # Module constants
    HL = length * 0.5
    HW = width * 0.5

    # Local robot coordinates
    LX = np.array([-HL, HL, HL, -HL], dtype=np.float32)
    LY = np.array([-HW, -HW, HW, HW], dtype=np.float32)

    # Extract columns
    x = pose[:, 0]
    y = pose[:, 1]
    theta = pose[:, 2]

    # Typedef
    c = np.cos(theta)
    s = np.sin(theta)

    # Create views once
    x = x[:, None]
    y = y[:, None]
    c = c[:, None]
    s = s[:, None]

    vx = vertices[:, :, 0]
    vy = vertices[:, :, 1]

    # Write directly into destination arrays
    np.multiply(c, LX, out=vx)
    vx -= s * LY
    vx += x

    np.multiply(s, LX, out=vy)
    vy += c * LY
    vy += y


def point_to_aabb(
    center_x: float,
    center_y: float,
    length: float,
    width: float,
    points: NDArray[np.float32],  # (N, 2)
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    dx = points[:, 0] - center_x
    dy = points[:, 1] - center_y

    hx = length * 0.5
    hy = width * 0.5

    closest_x = np.clip(dx, -hx, hx)
    closest_y = np.clip(dy, -hy, hy)

    offset_x = dx - closest_x
    offset_y = dy - closest_y

    distance = np.hypot(offset_x, offset_y)

    normal = np.empty((len(points), 2), dtype=np.float32)
    normal[:, 0] = offset_x / distance
    normal[:, 1] = offset_y / distance

    return distance, normal


def point_to_obb(
    pose: Pose,
    length: float,
    width: float,
    points: NDArray[np.float32],  # (N,2)
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:

    dx = points[:, 0] - pose.x
    dy = points[:, 1] - pose.y

    c = np.cos(pose.theta)
    s = np.sin(pose.theta)

    # Transform into rectangle frame
    local_x = dx * c + dy * s
    local_y = -dx * s + dy * c

    half_length = length * 0.5
    half_width = width * 0.5

    inside = (local_x >= -half_length) & (local_x <= half_length) & (local_y >= -half_width) & (local_y <= half_width)

    n = len(points)

    distance = np.empty(n, dtype=np.float32)

    local_normal = np.empty((n, 2), dtype=np.float32)

    # Outside points

    closest_x = np.clip(local_x, -half_length, half_length)
    closest_y = np.clip(local_y, -half_width, half_width)

    offset_x = local_x - closest_x
    offset_y = local_y - closest_y

    outside = ~inside

    distance[outside] = np.hypot(
        offset_x[outside],
        offset_y[outside],
    )

    nz = outside & (distance > 1e-9)

    local_normal[nz, 0] = offset_x[nz] / distance[nz]
    local_normal[nz, 1] = offset_y[nz] / distance[nz]

    normal = np.empty_like(local_normal)

    normal[:, 0] = local_normal[:, 0] * c - local_normal[:, 1] * s
    normal[:, 1] = local_normal[:, 0] * s + local_normal[:, 1] * c

    return distance, normal


def tangent(normal: NDArray[np.float32], heading: NDArray[np.float32]) -> NDArray[np.float32]:
    tg = np.empty_like(normal)
    tg[:, 0] = -normal[:, 1]
    tg[:, 1] = normal[:, 0]
    dot = np.sum(tg * heading, axis=1)
    tg[dot < 0] *= -1
    return tg


class SpatialHash:
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.cells = defaultdict(list)

    def clear(self):
        self.cells.clear()

    def _cell(self, x: float, y: float):
        return (
            int(x // self.cell_size),
            int(y // self.cell_size),
        )

    def insert(self, obj):
        self.cells[self._cell(obj.pose.x, obj.pose.y)].append(obj)

    def nearby(self, obj):
        cx, cy = self._cell(obj.pose.x, obj.pose.y)

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                yield from self.cells.get((cx + dx, cy + dy), [])


def obb_aabb_distance(
    obb_pose: NDArray[np.float32],
    obb_length: float,
    obb_width: float,
    center_x: float,
    center_y: float,
    length: float,
    width: float,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:

    pose = np.asarray(obb_pose, dtype=np.float32)

    N = pose.shape[0]

    zero = np.float32(0.0)
    one = np.float32(1.0)
    half = np.float32(0.5)

    cx = np.float32(center_x)
    cy = np.float32(center_y)

    hlb = half * np.float32(length)  # AABB half length
    hwb = half * np.float32(width)  # AABB half width
    hlr = half * np.float32(obb_length)  # robot half length
    hwr = half * np.float32(obb_width)  # robot half width

    # Output distance is also the running best squared distance.
    distance = np.empty(N, dtype=np.float32)

    # Fortran order makes normal[:, 0] and normal[:, 1] contiguous SoA arrays.
    # This avoids strided AoS updates on the hottest output buffer.
    normal = np.empty((N, 2), dtype=np.float32, order="F")
    best_nx = normal[:, 0]
    best_ny = normal[:, 1]

    # local[i] == True means the current best residual for i is expressed
    # in the robot OBB-local frame and must be rotated once at the end.
    # False means the current best residual is already in world coordinates.
    local = np.zeros(N, dtype=np.bool_)

    # Reused mask for running-min updates and final normalization.
    mask = np.empty(N, dtype=np.bool_)

    # One scratch buffer reduces allocator overhead and keeps temporaries local.
    #
    # Layout:
    #   0 px, 1 py, 2 c, 3 s,
    #   4 d2, 5 nx, 6 ny,
    #   7 t0, 8 t1, 9 t2, 10 t3, 11 t4, 12 t5
    work = np.empty((13, N), dtype=np.float32)

    px = work[0]
    py = work[1]
    c = work[2]
    s = work[3]

    d2 = work[4]
    nx = work[5]
    ny = work[6]

    t0 = work[7]
    t1 = work[8]
    t2 = work[9]
    t3 = work[10]
    t4 = work[11]
    t5 = work[12]

    pose_x = pose[:, 0]
    pose_y = pose[:, 1]
    pose_t = pose[:, 2]

    # Robot centers relative to AABB center. This reduces cancellation
    # compared with using absolute world coordinates directly.
    np.subtract(pose_x, cx, out=px)
    np.subtract(pose_y, cy, out=py)

    # Trig computed once.
    np.cos(pose_t, out=c)
    np.sin(pose_t, out=s)

    # ------------------------------------------------------------------
    # Candidate set 1: robot vertices -> AABB.
    #
    # Precompute the rotated half-extent contributions:
    #
    #   t0 = A = c * hlr
    #   t1 = B = s * hwr
    #   t2 = C = s * hlr
    #   t3 = D = c * hwr
    #
    # For local robot vertex (sx * hlr, sy * hwr):
    #
    #   ux = px + sx*A - sy*B
    #   uy = py + sx*C + sy*D
    #
    # This replaces per-vertex scalar multiplications with adds/subtracts.
    # ------------------------------------------------------------------
    np.multiply(c, hlr, out=t0)  # A
    np.multiply(s, hwr, out=t1)  # B
    np.multiply(s, hlr, out=t2)  # C
    np.multiply(c, hwr, out=t3)  # D

    first = True

    # Candidate order matches the original implementation:
    #   (+hlr, +hwr), (+hlr, -hwr), (-hlr, +hwr), (-hlr, -hwr)
    for sx_pos, sy_pos in (
        (True, True),
        (True, False),
        (False, True),
        (False, False),
    ):
        # ux = px + sx*A - sy*B
        if sx_pos:
            np.add(px, t0, out=t4)
        else:
            np.subtract(px, t0, out=t4)

        if sy_pos:
            np.subtract(t4, t1, out=t4)
        else:
            np.add(t4, t1, out=t4)

        # uy = py + sx*C + sy*D
        if sx_pos:
            np.add(py, t2, out=t5)
        else:
            np.subtract(py, t2, out=t5)

        if sy_pos:
            np.add(t5, t3, out=t5)
        else:
            np.subtract(t5, t3, out=t5)

        # Outside amounts relative to AABB:
        #   outside = max(abs(u) - half_extent, 0)
        np.abs(t4, out=nx)
        np.subtract(nx, hlb, out=nx)
        np.maximum(nx, zero, out=nx)

        np.abs(t5, out=ny)
        np.subtract(ny, hwb, out=ny)
        np.maximum(ny, zero, out=ny)

        # Candidate world residual from AABB closest point to robot vertex.
        # The signed outside vector points from the AABB toward the robot.
        np.copysign(nx, t4, out=nx)
        np.copysign(ny, t5, out=ny)

        # Squared distance. After copysign, t4 is no longer needed and can
        # be used as scratch for ny^2.
        np.multiply(nx, nx, out=d2)
        np.multiply(ny, ny, out=t4)
        np.add(d2, t4, out=d2)

        # Running minimum. The first candidate initializes the best values.
        if first:
            np.copyto(distance, d2)
            np.copyto(best_nx, nx)
            np.copyto(best_ny, ny)
            first = False
        else:
            np.less(d2, distance, out=mask)
            np.copyto(distance, d2, where=mask)
            np.copyto(best_nx, nx, where=mask)
            np.copyto(best_ny, ny, where=mask)
            # local remains False for all OBB-vertex candidates.

    # ------------------------------------------------------------------
    # Candidate set 2: AABB vertices -> robot OBB.
    #
    # Work in the robot OBB-local frame.
    #
    # Let p be the robot center relative to the AABB center.
    # We overwrite px/py with:
    #
    #   px = rlx =  c*px_old + s*py_old
    #   py = rly = -s*px_old + c*py_old
    #
    # For an AABB vertex q, the local coordinate is:
    #
    #   z = R^T(q - p) = R^T q - R^T p
    #
    # The four R^T q offsets are precomputed using symmetry:
    #
    #   XP =  clb + sbw
    #   XM =  clb - sbw
    #   YP = -slb + cbw
    #   YM = -slb - cbw
    #
    # The candidates are:
    #
    #   (++): z = ( XP - rlx,  YP - rly)
    #   (+-): z = ( XM - rlx,  YM - rly)
    #   (-+): z = (-XM - rlx, -YM - rly)
    #   (--): z = (-XP - rlx, -YP - rly)
    #
    # The residual for this candidate set is stored in robot-local
    # coordinates and rotated only at the end if it wins. This avoids
    # rotating losing candidates.
    # ------------------------------------------------------------------

    # Compute R^T p into px/py. Use t0..t3 as scratch before overwriting.
    np.multiply(c, px, out=t0)  # c * px_old
    np.multiply(s, px, out=t1)  # s * px_old
    np.multiply(s, py, out=t2)  # s * py_old
    np.multiply(c, py, out=t3)  # c * py_old

    np.add(t0, t2, out=px)  # rlx =  c*px + s*py
    np.subtract(t3, t1, out=py)  # rly = -s*px + c*py

    # Precompute AABB vertex offsets in robot-local coordinates.
    #
    # clb = c * hlb
    # sbw = s * hwb
    # slb = s * hlb
    # cbw = c * hwb
    np.multiply(c, hlb, out=t0)  # clb
    np.multiply(s, hwb, out=t1)  # sbw

    # XP = clb + sbw, XM = clb - sbw
    np.subtract(t0, t1, out=t2)  # XM -> t2
    np.add(t0, t1, out=t0)  # XP -> t0

    np.multiply(s, hlb, out=t1)  # slb
    np.multiply(c, hwb, out=t3)  # cbw

    # YP = cbw - slb, YM = -slb - cbw
    np.subtract(t3, t1, out=t4)  # YP -> t4
    np.add(t1, t3, out=t5)  # slb + cbw
    np.negative(t5, out=t5)  # YM -> t5

    # Offsets now live in:
    #   XP = t0
    #   XM = t2
    #   YP = t4
    #   YM = t5
    #
    # Robot-local center:
    #   rlx = px
    #   rly = py
    #
    # Free scratch for z_x/z_y:
    #   t1, t3

    # Candidate order matches the original implementation:
    #   (+hlb, +hwb), (+hlb, -hwb), (-hlb, +hwb), (-hlb, -hwb)
    for x_off, y_off, neg in (
        (t0, t4, False),  # (++): XP, YP
        (t2, t5, False),  # (+-): XM, YM
        (t2, t5, True),  # (-+): -XM, -YM
        (t0, t4, True),  # (--): -XP, -YP
    ):
        if neg:
            # z = -offset - r_local = -(offset + r_local)
            np.add(x_off, px, out=t1)
            np.negative(t1, out=t1)

            np.add(y_off, py, out=t3)
            np.negative(t3, out=t3)
        else:
            # z = offset - r_local
            np.subtract(x_off, px, out=t1)
            np.subtract(y_off, py, out=t3)

        # Outside amounts in robot-local coordinates:
        #   outside_local = max(abs(z) - robot_half_extent, 0)
        np.abs(t1, out=nx)
        np.subtract(nx, hlr, out=nx)
        np.maximum(nx, zero, out=nx)

        np.abs(t3, out=ny)
        np.subtract(ny, hwr, out=ny)
        np.maximum(ny, zero, out=ny)

        # Local residual from AABB vertex to closest point on robot OBB:
        #
        #   clamp(z) - z = -sign(z) * outside_amount
        np.copysign(nx, t1, out=nx)
        np.negative(nx, out=nx)

        np.copysign(ny, t3, out=ny)
        np.negative(ny, out=ny)

        # Squared distance. t1 is no longer needed after copysign.
        np.multiply(nx, nx, out=d2)
        np.multiply(ny, ny, out=t1)
        np.add(d2, t1, out=d2)

        # Running minimum. If this candidate wins, the residual is local.
        np.less(d2, distance, out=mask)
        np.copyto(distance, d2, where=mask)
        np.copyto(best_nx, nx, where=mask)
        np.copyto(best_ny, ny, where=mask)
        np.copyto(local, True, where=mask)

    # ------------------------------------------------------------------
    # Lazy rotation of winning AABB-vertex residuals.
    #
    # best_nx/best_ny currently contain either:
    #   - world residual, if local[i] == False
    #   - robot-local residual, if local[i] == True
    #
    # Rotate only the local ones. This is at most one rotation per query,
    # instead of up to four rotations per query.
    # ------------------------------------------------------------------
    if np.any(local):
        # wx = c*lx - s*ly
        np.multiply(c, best_nx, out=t0)
        np.multiply(s, best_ny, out=t1)
        np.subtract(t0, t1, out=t0)

        # wy = s*lx + c*ly
        np.multiply(s, best_nx, out=t1)
        np.multiply(c, best_ny, out=t2)
        np.add(t1, t2, out=t1)

        np.copyto(best_nx, t0, where=local)
        np.copyto(best_ny, t1, where=local)

    # ------------------------------------------------------------------
    # Normalize the winning residual vector.
    #
    # Use one reciprocal and two multiplications instead of two divisions.
    # ------------------------------------------------------------------
    np.sqrt(distance, out=distance)
    np.greater(distance, zero, out=mask)  # mask == valid non-zero distance

    # t0 is now used as inverse distance. Initialize to zero so that
    # zero-distance entries safely produce zero normals before fallback.
    t0.fill(zero)

    with np.errstate(divide="ignore", invalid="ignore"):
        np.divide(one, distance, out=t0, where=mask)

    np.multiply(best_nx, t0, out=best_nx)
    np.multiply(best_ny, t0, out=best_ny)

    return distance, normal
