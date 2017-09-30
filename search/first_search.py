# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

import heapq
from pprint import pprint


class PriorityQueueSet(object):

    """
    Combined priority queue and set data structure.

    Acts like a priority queue, except that its items are guaranteed to be
    unique. Provides O(1) membership test, O(log N) insertion and O(log N)
    removal of the smallest item.

    Important: the items of this data structure must be both comparable and
    hashable (i.e. must implement __cmp__ and __hash__). This is true of
    Python's built-in objects, but you should implement those methods if you
    want to use the data structure for custom objects.
    """
    def __init__(self):
        """Create a new PriorityQueueSet.

        Arguments:
            items (list): An initial item list - it can be unsorted and
                non-unique. The data structure will be created in O(N).
        """
        self.heap = []

    def has_item(self, item):
        """Check if ``item`` exists in the queue."""
        return item in self.heap

    def pop_smallest(self):
        """Remove and return the smallest item from the queue."""
        smallest = heapq.heappop(self.heap)
        return smallest

    def add(self, item):
        """Add ``item`` to the queue if doesn't already exist."""
        if item not in self.heap:
            heapq.heappush(self.heap, item)

    def empty(self):
        if len(self.heap) == 0:
            return True

        return False

class GridLoc:
    def __init__(self, row, col):
        self.row = row
        self.col = col

    def __str__(self):
        return (self.row, self.col)

    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    def __hash__(self):
        return (self.row, self.col).__hash__()

    def new_pos(self, row, col):
        return GridLoc(self.row + row, self.col + col)

    def __lt__(self, other):
        return self.row < other.row and self.col < other.col

    def __repr__(self):
        return (self.row, self.col)


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = GridLoc(len(grid)-1, len(grid[0])-1)
cost = 1

delta = [[-1,  0],  # go up
         [0, -1],  # go left
         [1,  0],  # go down
         [0,  1]]  # go right

delta_name = ['^', '<', 'v', '>']


def _is_valid_grid_loc(grid, grid_loc):
    if grid_loc.row < 0 or grid_loc.col < 0 or grid_loc.row >= len(grid) or grid_loc.col >= len(grid[0]):
        return False

    return True

def _is_blocked(grid, grid_loc):
    if grid[grid_loc.row][grid_loc.col] == 1:
        return True

    return False


def search(grid, init, goal, cost):
    pq = PriorityQueueSet()
    pq.add((0, GridLoc(init[0], init[1])))
    visited = set()
    visited.add(GridLoc(init[0], init[1]))
    current = None
    expansion = [[-1 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    idx = 0
    while not pq.empty():
        current = pq.pop_smallest()
        current_cost = current[0]
        current_pos = current[1]
        print("Exploring: (%s) --> (%s, %s)" % (current_cost, current_pos.row, current_pos.col))
        expansion[current_pos.row][current_pos.col] = idx
        idx += 1
        visited.add(current_pos)
        if current_pos == goal:
            break

        for move in delta:
            new_pos = current_pos.new_pos(move[0], move[1])
            if _is_valid_grid_loc(grid, new_pos) and new_pos not in visited and not _is_blocked(grid, new_pos):
                pq.add((current_cost + 1, new_pos))

        current = None

    if current is None:
        return "fail"

    path = [current[0], current[1].row, current[1].col]
    return path, expansion

def main():
    pprint(search(grid, init, goal, cost))


if __name__ == "__main__":
    main()
