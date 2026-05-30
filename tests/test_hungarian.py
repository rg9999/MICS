import numpy as np

from mics import hungarian


def total(cost, pairs):
    return sum(cost[r, c] for r, c in pairs)


def test_square_optimal():
    cost = np.array([[4, 1, 3], [2, 0, 5], [3, 2, 2]], dtype=float)
    pairs = hungarian.solve(cost)
    assert len(pairs) == 3
    # known optimal total for this classic matrix is 5
    assert total(cost, pairs) == 5


def test_rectangular_more_drones_than_targets():
    # 4 drones, 2 targets -> only 2 assignments, lowest total
    cost = np.array([
        [10, 19],
        [3, 8],
        [5, 4],
        [12, 7],
    ], dtype=float)
    pairs = hungarian.solve(cost)
    assert len(pairs) == 2
    cols = sorted(c for _, c in pairs)
    assert cols == [0, 1]
    # optimal: drone1->t0 (3), drone2->t1 (4) = 7
    assert total(cost, pairs) == 7


def test_rectangular_more_targets_than_drones():
    cost = np.array([
        [4, 2, 8, 1],
        [7, 3, 5, 6],
    ], dtype=float)
    pairs = hungarian.solve(cost)
    assert len(pairs) == 2  # only 2 drones can be assigned
    rows = sorted(r for r, _ in pairs)
    assert rows == [0, 1]


def test_matches_brute_force_small():
    rng = np.random.default_rng(0)
    from itertools import permutations
    for _ in range(20):
        n = 4
        cost = rng.uniform(0, 100, size=(n, n))
        pairs = hungarian.solve(cost)
        got = total(cost, pairs)
        best = min(sum(cost[i, p[i]] for i in range(n))
                   for p in permutations(range(n)))
        assert abs(got - best) < 1e-6
