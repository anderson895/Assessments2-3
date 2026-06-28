"""
CAI104 - Concepts in AI
Assessment 3: Robot Path Planner
Author: ____________________

This program implements an A* (A-star) search to find the optimal (shortest)
path for a robot navigating a 2D grid maze from its initial position to a
target cell.

Grid legend (as given in the assessment brief):
     1  -> wall
     0  -> space (void / walkable)
    -1  -> initial position of the robot (start)
     9  -> the target (goal)

The shortest path returned by the algorithm is marked on the map with the
value 2 (any number other than 1, 0, -1 and 9 may be used).

Why A*?
    A* is an INFORMED search algorithm. It expands the node that minimises
        f(n) = g(n) + h(n)
    where g(n) is the real cost from the start to node n, and h(n) is an
    admissible heuristic estimating the remaining cost from n to the goal.
    With the Manhattan distance heuristic on a 4-connected grid (which never
    overestimates the true cost) A* is guaranteed to return the optimal path
    while expanding far fewer nodes than an uninformed search such as BFS.
"""

import heapq

# ---------------------------------------------------------------------------
# 1. PROBLEM DEFINITION (the maze exactly as provided in the assessment brief)
# ---------------------------------------------------------------------------
# 12 rows x 24 columns = 288 cells.
MAZE = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 9, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]

WALL, VOID, START, GOAL, PATH = 1, 0, -1, 9, 2

# 4-connected movement: Up, Down, Left, Right (no diagonals).
MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# ---------------------------------------------------------------------------
# 2. HELPER FUNCTIONS
# ---------------------------------------------------------------------------
def find_cell(maze, value):
    """Return the (row, col) of the first cell holding `value`, else None."""
    for r, row in enumerate(maze):
        for c, cell in enumerate(row):
            if cell == value:
                return (r, c)
    return None


def manhattan(a, b):
    """Admissible heuristic for a 4-connected grid: |dr| + |dc|."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbours(maze, cell):
    """Yield walkable neighbours (not walls, inside the grid)."""
    rows, cols = len(maze), len(maze[0])
    for dr, dc in MOVES:
        nr, nc = cell[0] + dr, cell[1] + dc
        if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] != WALL:
            yield (nr, nc)


# ---------------------------------------------------------------------------
# 3. A* SEARCH
# ---------------------------------------------------------------------------
def a_star(maze, start, goal):
    """
    Run A* search. Returns (path, expanded) where `path` is the list of cells
    from start to goal (inclusive) or None if unreachable, and `expanded` is
    the number of nodes popped from the frontier (a measure of efficiency).
    """
    # Priority queue ordered by f = g + h. Entries: (f, g, cell).
    frontier = [(manhattan(start, goal), 0, start)]
    came_from = {start: None}      # child -> parent, to rebuild the path
    g_score = {start: 0}           # best known cost from start to each cell
    expanded = 0

    while frontier:
        f, g, current = heapq.heappop(frontier)
        expanded += 1

        # Goal test on expansion guarantees the optimal path with an
        # admissible heuristic.
        if current == goal:
            return reconstruct(came_from, goal), expanded

        # Skip stale queue entries (a better g was found after queueing).
        if g > g_score[current]:
            continue

        for nxt in neighbours(maze, current):
            tentative_g = g + 1          # every step costs 1
            if nxt not in g_score or tentative_g < g_score[nxt]:
                g_score[nxt] = tentative_g
                came_from[nxt] = current
                f_next = tentative_g + manhattan(nxt, goal)
                heapq.heappush(frontier, (f_next, tentative_g, nxt))

    return None, expanded            # goal not reachable


def reconstruct(came_from, goal):
    """Walk the parent pointers back from goal to start, then reverse."""
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# 4. VISUALISATION
# ---------------------------------------------------------------------------
def mark_and_print(maze, path):
    """Print the maze with the path marked by `2` (start/goal preserved)."""
    grid = [row[:] for row in maze]           # deep copy
    for (r, c) in path:
        if grid[r][c] == VOID:                # keep START (-1) and GOAL (9)
            grid[r][c] = PATH

    symbols = {WALL: "#", VOID: " ", START: "S", GOAL: "G", PATH: "."}
    print("\nMaze with optimal path ('.' = path, S = start, G = goal):\n")
    for row in grid:
        print("".join(symbols[c] for c in row))

    print("\nRaw 2D array (path = 2):\n")
    for row in grid:
        print("{" + ",".join(f"{v:2d}" for v in row) + "},")


# ---------------------------------------------------------------------------
# 5. MAIN
# ---------------------------------------------------------------------------
def main():
    start = find_cell(MAZE, START)
    goal = find_cell(MAZE, GOAL)
    print(f"Start (robot): {start}")
    print(f"Goal  (target): {goal}")

    path, expanded = a_star(MAZE, start, goal)

    if path is None:
        print("\nNo path exists between the start and the target.")
        return

    print(f"\nA* expanded {expanded} nodes.")
    print(f"Optimal path length: {len(path)} cells "
          f"({len(path) - 1} moves).")
    print(f"Path coordinates:\n{path}")
    mark_and_print(MAZE, path)


if __name__ == "__main__":
    main()
