"""Hungarian algorithm (Kuhn-Munkres) for optimal one-to-one assignment.

Pure-numpy implementation (no scipy). Minimises total cost over a rectangular
cost matrix; pads to square internally. Returns a list of (row, col) pairs for
the assigned rows. Used by mics_allocator for v1 optimal allocation (PRD §7.2).
"""

from __future__ import annotations

import numpy as np

_BIG = 1e9


def solve(cost: np.ndarray) -> list[tuple[int, int]]:
    """Return optimal (row, col) assignments minimising total cost.

    cost: (n_rows x n_cols). Rows/cols that are padded (no real counterpart)
    are dropped from the result.
    """
    cost = np.asarray(cost, dtype=float)
    n_rows, n_cols = cost.shape
    n = max(n_rows, n_cols)
    # pad to square with large cost
    C = np.full((n, n), 0.0)
    C[:n_rows, :n_cols] = cost
    # entries outside the real matrix get BIG so they're never preferred
    if n_cols < n:
        C[:, n_cols:] = _BIG
    if n_rows < n:
        C[n_rows:, :] = _BIG

    assignment = _munkres(C)
    result = []
    for r, c in enumerate(assignment):
        if r < n_rows and c < n_cols and cost[r, c] < _BIG / 2:
            result.append((r, c))
    return result


def _munkres(C: np.ndarray) -> list[int]:
    n = C.shape[0]
    C = C.copy()
    # Step 1: subtract row minima
    C -= C.min(axis=1, keepdims=True)
    # Step 2: subtract column minima
    C -= C.min(axis=0, keepdims=True)

    starred = np.zeros((n, n), dtype=bool)
    primed = np.zeros((n, n), dtype=bool)
    row_cov = np.zeros(n, dtype=bool)
    col_cov = np.zeros(n, dtype=bool)

    # Step 3: star an arbitrary independent zero
    for i in range(n):
        for j in range(n):
            if C[i, j] == 0 and not row_cov[i] and not col_cov[j]:
                starred[i, j] = True
                row_cov[i] = True
                col_cov[j] = True
    row_cov[:] = False
    col_cov[:] = False

    def cover_starred_cols():
        col_cov[:] = starred.any(axis=0)
        return col_cov.sum()

    while cover_starred_cols() < n:
        while True:
            # find an uncovered zero
            z = _find_uncovered_zero(C, row_cov, col_cov)
            if z is None:
                # Step 6: adjust matrix by smallest uncovered value
                minval = _min_uncovered(C, row_cov, col_cov)
                C[~row_cov, :] -= minval
                C[:, col_cov] += minval
                continue
            i, j = z
            primed[i, j] = True
            star_col = _star_in_row(starred, i)
            if star_col is None:
                _augment(starred, primed, i, j)
                primed[:] = False
                row_cov[:] = False
                col_cov[:] = False
                break
            else:
                row_cov[i] = True
                col_cov[star_col] = False

    return [int(np.where(starred[i])[0][0]) for i in range(n)]


def _find_uncovered_zero(C, row_cov, col_cov):
    for i in range(C.shape[0]):
        if row_cov[i]:
            continue
        for j in range(C.shape[1]):
            if not col_cov[j] and C[i, j] == 0:
                return (i, j)
    return None


def _min_uncovered(C, row_cov, col_cov):
    mask = np.ones_like(C, dtype=bool)
    mask[row_cov, :] = False
    mask[:, col_cov] = False
    return C[mask].min()


def _star_in_row(starred, i):
    cols = np.where(starred[i])[0]
    return int(cols[0]) if len(cols) else None


def _star_in_col(starred, j):
    rows = np.where(starred[:, j])[0]
    return int(rows[0]) if len(rows) else None


def _prime_in_row(primed, i):
    cols = np.where(primed[i])[0]
    return int(cols[0]) if len(cols) else None


def _augment(starred, primed, i, j):
    # build alternating path of primes and stars (Step 5)
    path = [(i, j)]
    while True:
        r = _star_in_col(starred, path[-1][1])
        if r is None:
            break
        path.append((r, path[-1][1]))
        c = _prime_in_row(primed, r)
        path.append((r, c))
    for (pi, pj) in path:
        if starred[pi, pj]:
            starred[pi, pj] = False
        else:
            starred[pi, pj] = True
