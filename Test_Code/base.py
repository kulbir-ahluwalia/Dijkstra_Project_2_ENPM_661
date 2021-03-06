import matplotlib.pyplot as plt
import math
from math import sqrt
show_animation = True

class Dijkstra:
    def __init__(self, ox, oy, resolution, radius, clearance):
        self.min_x = 0
        self.min_y = 0
        self.max_x = 300
        self.max_y = 200
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution #grid resolution
        self.radius = radius  #robot radius
        self.clearance = clearance  #clearance for obstacles
        self.calc_obstacle_map(ox, oy) #position list of obstacles
        self.motion = self.motion_model()

    class Node:
        def __init__(self, x, y, cost, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent = parent  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        empty_set = dict()
        visited_set = dict()

        empty_set[self.calc_index(start_node)] = start_node

        while 1:
            c_node = min(empty_set, key=lambda c: empty_set[c].cost)   #the key whose value is the smallest
            current = empty_set[c_node]
            # print("The key with the smallest cost:", c_node)
            # print("The smallest cost:", empty_set[c_node])
            # print(empty_set)
            # show animation
            if show_animation:
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), ".y")
                if len(visited_set.keys()) % 1 == 0:      # control the speed of animation
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break


            del empty_set[c_node]      # Remove the item from the empty set
            visited_set[c_node] = current   # Add it to the visited set

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_node)
                n_node = self.calc_index(node)

                if n_node in visited_set:
                    continue

                if n_node not in empty_set:
                    empty_set[n_node] = node  # Discover a new node

                if not self.verify_node(node):
                    continue

                else:
                    if empty_set[n_node].cost >= node.cost:
                        empty_set[n_node] = node

        rx, ry = self.calc_final_path(goal_node, visited_set)

        return rx, ry

    def calc_final_path(self, goal_node, visited_set):  #Generate path
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != -1: #########
            n = visited_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        # self.min_x = round(min(ox))
        # self.min_y = round(min(oy))
        # self.max_x = round(max(ox))
        # self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = sqrt((iox - x)**2 + (ioy - y)**2)
                    if d <= (self.radius + self.clearance):
                        self.obstacle_map[ix][iy] = True
                        break

    def motion_model(self):
        # dx, dy, cost
        motion = [[1, 0, 1],                #Move Right
                  [1, 1, sqrt(2)],     #Move Right Up
                  [0, 1, 1],                #Move Up
                  [-1, 1, sqrt(2)],    #Move Left Up
                  [-1, 0, 1],               #Move Left
                  [-1, -1, sqrt(2)],   #Move Left Down
                  [0, -1, 1],               #Move Down
                  [1, -1, sqrt(2)]]    #Move Left Down

        return motion


def main():

    # start and goal position
    sx = int(input("Enter the starting point x: \n"))
    sy = int(input("Enter the starting point y: \n"))
    gx = int(input("Enter the goal point x: \n"))
    gy = int(input("Enter the goal point y: \n"))
    resolution = 4
    radius = 2
    clearance = 2

    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 300):
        ox.append(i)
        oy.append(0)
    for i in range(0, 200):
        ox.append(300)
        oy.append(i)
    for i in range(0, 300):
        ox.append(i)
        oy.append(200)
    for i in range(0, 200):
        ox.append(0)
        oy.append(i)


    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(False)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, resolution, radius, clearance)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()