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
        self.set = set()
        self.heap = []

    def has_item(self, item):
        """Check if ``item`` exists in the queue."""
        return item in self.set

    def pop_smallest(self):
        """Remove and return the smallest item from the queue."""
        smallest = heapq.heappop(self.heap)
        self.set.remove(smallest)
        return smallest

    def add(self, item):
        """Add ``item`` to the queue if doesn't already exist."""
        if item not in self.set:
            self.set.add(item)
            heapq.heappush(self.heap, item)

    def empty(self):
        if len(self.heap) == 0:
            return True

        return False


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1,  0],  # go up
         [0, -1],  # go left
         [1,  0],  # go down
         [0,  1]]  # go right

delta_name = ['^', '<', 'v', '>']


def _is_valid_grid_loc(grid, row, col):
    if row < 0 or col < 0 or row >= len(grid) or col >= len(grid[0]):
        return False

    return True


def search(grid, init, goal, cost):
    path = []

    pq = PriorityQueueSet()
    pq.add((0, init))
    visited = set()
    visited.add(init)
    while not pq.empty():
        current = pq.pop_smallest()
        current_cost = current[0]
        current_pos = current[1]
        if current_pos == goal:
            break

        for move in delta:
            new_pos = current_pos + move
            if _is_valid_grid_loc(grid, new_pos[0], new_pos[1]) and new_pos not in visited:
                pq.add((current_cost + 1, current_pos))

        current = None

    if current is None:
        return "fail"

    path = [current[0], current[1][0], current[1][1]]

    return path


def main():
    search(grid, init, goal, cost)


if __name__ == "__main__":
    main()
