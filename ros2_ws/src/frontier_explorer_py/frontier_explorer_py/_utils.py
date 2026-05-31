"""
Pure-Python / NumPy frontier utilities.  No ROS imports — fully testable offline.

Coordinate convention
---------------------
All positions are represented as integer voxel keys (ix, iy, iz) obtained by
rounding (x / resolution) to the nearest integer.  This avoids floating-point
equality issues when checking set membership.
"""

import numpy as np

_FACE_DIRS = [(1, 0, 0), (-1, 0, 0),
              (0, 1, 0), (0, -1, 0),
              (0, 0, 1), (0, 0, -1)]


def extract_frontiers(free_keys, known_keys):
    """Return frontier voxel keys.

    Parameters
    ----------
    free_keys  : iterable of (int, int, int) – free voxel grid keys
    known_keys : set of (int, int, int)      – all known voxels (free + occupied)

    Returns
    -------
    list of (int, int, int)
        Free voxels that have at least one unknown (not-in-known_keys) face neighbour.
    """
    known = known_keys if isinstance(known_keys, set) else set(known_keys)
    frontiers = []
    for k in free_keys:
        for d in _FACE_DIRS:
            nb = (k[0] + d[0], k[1] + d[1], k[2] + d[2])
            if nb not in known:
                frontiers.append(k)
                break
    return frontiers


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
