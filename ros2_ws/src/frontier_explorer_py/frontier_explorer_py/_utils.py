"""
Pure-Python / NumPy frontier utilities.  No ROS imports — fully testable offline.

Coordinate convention
---------------------
All positions are represented as integer voxel keys (ix, iy, iz) obtained by
rounding (x / resolution) to the nearest integer.  This avoids floating-point
equality issues when checking set membership.
"""

import numpy as np

_FACE_DIRS = np.array([
    [1, 0, 0], [-1, 0, 0],
    [0, 1, 0], [0, -1, 0],
    [0, 0, 1], [0, 0, -1],
], dtype=np.int32)

# Encoding constants: offset+scale must cover the full OctoMap key range
# (default depth 16 → keys in [-32768, 32767]).
_ENC_OFF = np.int64(32768)
_ENC_S   = np.int64(65536)   # 2^16, one step per axis


def _encode(keys: np.ndarray) -> np.ndarray:
    """Encode (N,3) int32 keys to unique int64 scalars for np.isin."""
    k = np.asarray(keys, dtype=np.int64)
    return (k[:, 0] + _ENC_OFF) * _ENC_S * _ENC_S + \
           (k[:, 1] + _ENC_OFF) * _ENC_S + \
           (k[:, 2] + _ENC_OFF)


def extract_frontiers(free_keys, known_keys):
    """Return frontier voxel keys using vectorised np.isin — O(N log M).

    Parameters
    ----------
    free_keys  : (N,3) int np.ndarray  OR  iterable of (int,int,int)
    known_keys : (M,3) int np.ndarray  OR  set/iterable of (int,int,int)
        All known voxels (free + occupied).

    Returns
    -------
    list of (int, int, int)
        Free voxels that have at least one unknown face neighbour.
    """
    # -- normalise to numpy --
    if isinstance(free_keys, np.ndarray) and free_keys.ndim == 2:
        fk = free_keys.astype(np.int32)
    else:
        lst = list(free_keys)
        fk = np.array(lst, dtype=np.int32) if lst else np.empty((0, 3), dtype=np.int32)

    if len(fk) == 0:
        return []

    if isinstance(known_keys, np.ndarray) and known_keys.ndim == 2:
        kk = known_keys.astype(np.int32)
    else:
        lst = list(known_keys)
        kk = np.array(lst, dtype=np.int32) if lst else np.empty((0, 3), dtype=np.int32)

    known_enc = _encode(kk) if len(kk) > 0 else np.empty(0, dtype=np.int64)

    frontier_mask = np.zeros(len(fk), dtype=bool)
    for d in _FACE_DIRS:
        nb_enc = _encode(fk + d)
        frontier_mask |= ~np.isin(nb_enc, known_enc)

    return [tuple(int(x) for x in row) for row in fk[frontier_mask]]


def cluster_frontiers(pts, cluster_radius, min_cluster_size):
    """Greedy radius clustering.  O(N·K) where K = number of clusters.

    Parameters
    ----------
    pts              : array-like (N, 3) float – frontier positions in metres
    cluster_radius   : float – merge radius in metres
    min_cluster_size : int   – drop clusters smaller than this

    Returns
    -------
    centroids : list of np.ndarray (3,) – one per surviving cluster
    labels    : np.ndarray (N,) int32  – cluster index per point, -1 if filtered
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) == 0:
        return [], np.empty(0, dtype=np.int32)

    r2 = cluster_radius ** 2
    centroids = []
    sizes = []
    labels = np.full(len(pts), -1, dtype=np.int32)

    for i, p in enumerate(pts):
        best_idx, best_d2 = -1, r2
        for ci, c in enumerate(centroids):
            d2 = float(np.sum((p - c) ** 2))
            if d2 < best_d2:
                best_d2, best_idx = d2, ci
        if best_idx >= 0:
            n = sizes[best_idx] + 1
            centroids[best_idx] = centroids[best_idx] + (p - centroids[best_idx]) / n
            sizes[best_idx] = n
            labels[i] = best_idx
        else:
            labels[i] = len(centroids)
            centroids.append(p.copy())
            sizes.append(1)

    # Drop small clusters and remap labels to contiguous indices.
    remap = {}
    kept = []
    for ci, (c, s) in enumerate(zip(centroids, sizes)):
        if s >= min_cluster_size:
            remap[ci] = len(kept)
            kept.append(c)
    for i in range(len(labels)):
        labels[i] = remap.get(int(labels[i]), -1)

    return kept, labels


def positions_to_keys(positions, resolution):
    """Convert an (N, 3) float array to an (N, 3) int32 array of grid keys."""
    return np.round(np.asarray(positions, dtype=np.float64) / resolution).astype(np.int32)
