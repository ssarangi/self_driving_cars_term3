import collections
import numpy as np
import math

NUM_THETA_CELLS = 90
SPEED = 1.45
LENGTH = 0.5
MAZE_S = collections.namedtuple("MAZE_S", ["g", "x", "y", "theta"])

def initialize_3d_array(initial_value, x_size, y_size, z_size):
    arr = [[[initial_value for _ in range(z_size)] for _ in range(y_size)] for _ in range(x_size)]
    return arr

class MazePath:
    def __init__(self, closed:[[]]=None, came_from:[[]]=None, final:MAZE_S=None):
        self.closed = closed
        self.came_from = came_from
        self.final = final

class HBF:
    def __init__(self):
        self.NUM_THETA_CELLS = 90
        self.SPEED = 1.45
        self.LENGTH = 0.5

    def theta_to_stack_number(self, theta):
        """
        Take an angle in radians and returns which stack in the 3D configuration space this angle corresponds to. Angles
        near 0 go in the lower stacks while angles near 2 * PI go in the higher stacks
        :param theta: Angle
        :return: Stack number
        """
        new_theta = (theta + 2 * np.pi) % ( 2 * np.pi)
        stack_number = int(round((new_theta * NUM_THETA_CELLS / (2 * np.pi)))) % NUM_THETA_CELLS
        return stack_number

    def idx(self, float_num: float):
        return int(math.floor(float_num))

    def expand(self, state : MAZE_S):
        g2 = state.g + 1
        next_states = []
        for delta_i in range(-35, 40, 5):
            delta = np.pi / 180.0 * delta_i
            omega = SPEED / LENGTH * math.tan(delta)
            theta2 = state.theta + omega
            if theta2 > 0:
                theta2 += 2 * np.pi

            x2 = state.x + SPEED * math.cos(theta2)
            y2 = state.y + SPEED * math.sin(theta2)
            state2 = MAZE_S(g=g2, x=x2, y=y2, theta=theta2)
            next_states.append(state2)

        return next_states


    def search(self, grid, start: [], goal: []):
        theta = start[2]
        stack = self.theta_to_stack_number(theta)

        closed = initialize_3d_array(0, NUM_THETA_CELLS, len(grid[0]), len(grid))
        closed_value = initialize_3d_array(0, NUM_THETA_CELLS, len(grid[0]), len(grid))
        came_from = initialize_3d_array(0, NUM_THETA_CELLS, len(grid[0]), len(grid))

        g = 0
        state = MAZE_S(g=g, x=start[0], y=start[1], theta=0)
        closed[stack][self.idx(state.x)][self.idx(state.y)] = state
        closed_value[stack][self.idx(state.x)][self.idx(state.y)] = 1
        came_from[stack][self.idx(state.x)][self.idx(state.y)] = state
        total_closed = 1
        opened = [state]
        finished = False
        while opened:
            next = opened[0]
            # Pop the first element
            opened = opened[1:]
            x, y = next.x, next.y

            if self.idx(x) == goal[0] and self.idx(y) == goal[1]:
                print("Found path to goal in %s expansions" % total_closed)
                path = MazePath(closed=closed, came_from=came_from, final=next)
                return path

            next_state = self.expand(next)
            for i in range(0, len(next_state)):
                g2 = next_state[i].g
                x2 = next_state[i].x
                y2 = next_state[i].y
                theta2 = next_state[i].theta

                if x2 < 0 or x >= len(grid) or y2 < 0 or y2 >= len(grid[0]):
                    # Invalid cell
                    continue

                stack2 = self.theta_to_stack_number(theta2)

                if closed_value[stack2][self.idx(x2)][self.idx(y2)] == 0 and grid[self.idx(x2)][self.idx(y2)] == '_':
                    state2 = MAZE_S(g=g2, x=x2, y=y2, theta=theta2)
                    opened.append(state2)

                    closed[stack2][self.idx(x2)][self.idx(y2)] = next_state[i]
                    closed_value[stack2][self.idx(x2)][self.idx(y2)] = 1
                    came_from[stack2][self.idx(x2)][self.idx(y2)] = next
                    total_closed += 1


        print("No Valid path found")
        path = MazePath(closed=closed, came_from=came_from, final=state)
        return path

    def reconstruct_path(self, came_from: [[[]]], start: [], final: MAZE_S):
        path = [final]
        stack = self.theta_to_stack_number(final.theta)
        current = came_from[stack][self.idx(final.x)][self.idx(final.y)]
        stack = self.theta_to_stack_number(current.theta)

        x, y = current.x, current.y
        while x != start[0] and y != start[1]:
            path.append(current)
            current = came_from[stack][self.idx(x)][self.idx(y)]
            x = current.x
            y = current.y
            stack = self.theta_to_stack_number(current.theta)

        return path