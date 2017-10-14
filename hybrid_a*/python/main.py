from hbf import HBF

MAZE = [
    ["_", "X", "X", "_", "_", "_", "_", "_", "_", "_", "X", "X", "_", "_", "_", "_", ],
    ["_", "X", "X", "_", "_", "_", "_", "_", "_", "X", "X", "_", "_", "_", "_", "_", ],
    ["_", "X", "X", "_", "_", "_", "_", "_", "X", "X", "_", "_", "_", "_", "_", "_", ],
    ["_", "X", "X", "_", "_", "_", "_", "X", "X", "_", "_", "_", "X", "X", "X", "_", ],
    ["_", "X", "X", "_", "_", "_", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", ],
    ["_", "X", "X", "_", "_", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", "_", ],
    ["_", "X", "X", "_", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", "_", "_", ],
    ["_", "X", "X", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", "_", "_", "_", ],
    ["_", "X", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", "_", "_", "_", "_", ],
    ["_", "X", "X", "_", "_", "_", "X", "X", "X", "_", "_", "X", "X", "X", "X", "X", ],
    ["_", "X", "_", "_", "_", "X", "X", "X", "_", "_", "X", "X", "X", "X", "X", "X", ],
    ["_", "_", "_", "_", "X", "X", "X", "_", "_", "X", "X", "X", "X", "X", "X", "X", ],
    ["_", "_", "_", "X", "X", "X", "_", "_", "X", "X", "X", "X", "X", "X", "X", "X", ],
    ["_", "_", "X", "X", "X", "_", "_", "X", "X", "X", "X", "X", "X", "X", "X", "X", ],
    ["_", "X", "X", "X", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", ],
    ["X", "X", "X", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", "_", ],
]

START = [0.0, 0.0, 0.0]
GOAL = (len(MAZE[0][0]) - 1, len(MAZE[0]))

def main():
    print("Finding path through grid: ")

    for i in range(0, len(MAZE)):
        print(" ".join(MAZE[i]))

    hbf = HBF()
    get_path = hbf.search(MAZE, START, GOAL)
    show_path = hbf.reconstruct_path(get_path.came_from, START, get_path.final)

    print("Show path from start to finish")
    for i in range(len(show_path), 0, -1):
        step = show_path[i]
        MAZE[step.x][step.y] = '.'

    for i in range(0, len(MAZE)):
        print(" ".join(MAZE[i]))

if __name__ == "__main__":
    main()