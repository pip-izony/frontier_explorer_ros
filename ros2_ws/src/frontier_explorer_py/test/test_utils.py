"""Unit tests for frontier_explorer_py._utils — no ROS required."""

import unittest
import numpy as np
from frontier_explorer_py._utils import extract_frontiers, cluster_frontiers, positions_to_keys


class TestExtractFrontiers(unittest.TestCase):

    def test_empty_map_returns_empty(self):
        self.assertEqual(extract_frontiers([], set()), [])

    def test_isolated_free_voxel_is_frontier(self):
        # One free voxel with all 6 face neighbours unknown.
        free  = [(0, 0, 0)]
        known = {(0, 0, 0)}
        result = extract_frontiers(free, known)
        self.assertEqual(len(result), 1)
        self.assertIn((0, 0, 0), result)

    def test_free_surrounded_by_occupied_is_not_frontier(self):
        free = [(0, 0, 0)]
        occ  = {(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)}
        known = {(0, 0, 0)} | occ
        self.assertEqual(extract_frontiers(free, known), [])

    def test_interior_voxel_not_a_frontier(self):
        free  = [(0,0,0),(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]
        known = set(free)
        result = extract_frontiers(free, known)
        self.assertNotIn((0, 0, 0), result)   # interior must be excluded
        self.assertEqual(len(result), 6)       # outer ring are all frontiers

    def test_occupied_voxels_not_returned(self):
        # Only occupied voxels in known — nothing in free → no frontiers.
        known = {(0,0,0),(1,0,0)}
        self.assertEqual(extract_frontiers([], known), [])

    def test_partial_neighbourhood_still_frontier(self):
        # One face is occupied, the other 5 are unknown → still a frontier.
        free  = [(0, 0, 0)]
        known = {(0, 0, 0), (1, 0, 0)}   # only +x is known/occupied
        result = extract_frontiers(free, known)
        self.assertEqual(len(result), 1)


class TestClusterFrontiers(unittest.TestCase):

    def test_empty_input(self):
        centroids, labels = cluster_frontiers([], 1.0, 1)
        self.assertEqual(centroids, [])
        self.assertEqual(len(labels), 0)

    def test_single_point(self):
        centroids, labels = cluster_frontiers([[0.0, 0.0, 0.0]], 1.0, 1)
        self.assertEqual(len(centroids), 1)
        self.assertEqual(labels[0], 0)

    def test_two_distant_groups_form_two_clusters(self):
        grp_a = [[i * 0.1, 0.0, 0.0] for i in range(5)]
        grp_b = [[20.0 + i * 0.1, 0.0, 0.0] for i in range(5)]
        centroids, labels = cluster_frontiers(grp_a + grp_b, 1.0, 3)
        self.assertEqual(len(centroids), 2)
        self.assertEqual(len(set(labels[:5])), 1)   # all group A share one label
        self.assertEqual(len(set(labels[5:])), 1)   # all group B share one label
        self.assertNotEqual(labels[0], labels[5])

    def test_small_cluster_filtered(self):
        pts = [[i * 0.1, 0.0, 0.0] for i in range(4)]   # 4 points, min_size=5
        centroids, labels = cluster_frontiers(pts, 1.0, 5)
        self.assertEqual(len(centroids), 0)
        self.assertTrue(all(l == -1 for l in labels))

    def test_centroid_accuracy(self):
        pts = [[-0.1, 0.0, 0.0], [0.0, 0.0, 0.0], [0.1, 0.0, 0.0]]
        centroids, _ = cluster_frontiers(pts, 1.0, 1)
        self.assertEqual(len(centroids), 1)
        self.assertAlmostEqual(float(centroids[0][0]), 0.0, places=6)

    def test_labels_contiguous_after_filter(self):
        # Two groups both ≥ min_size → labels must be 0 and 1.
        grp_a = [[i * 0.1, 0, 0] for i in range(3)]
        grp_b = [[10 + i * 0.1, 0, 0] for i in range(3)]
        _, labels = cluster_frontiers(grp_a + grp_b, 1.0, 2)
        self.assertEqual(set(labels), {0, 1})


class TestPositionsToKeys(unittest.TestCase):

    def test_round_trip(self):
        res = 0.1
        pos = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [-0.1, 0.0, 0.0]])
        keys = positions_to_keys(pos, res)
        self.assertEqual(keys[0].tolist(), [0, 0, 0])
        self.assertEqual(keys[1].tolist(), [1, 0, 0])
        self.assertEqual(keys[2].tolist(), [-1, 0, 0])


if __name__ == '__main__':
    unittest.main()
