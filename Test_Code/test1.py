import matplotlib.pyplot as plt
import math
from math import sqrt
show_animation = True

class Dijkstra:
    def __init__(self, ox, oy, radius, clearance):
        self.min_x = 0
        self.min_y = 0
        self.max_x = 300
        self.max_y = 200
        self.grid_size = 4
        self.x_width = 300/4
        self.y_width = 200/4
        self.obstacle_map = None

        self.radius = radius   #robot radius
        self.clearance = clearance  #clearance for obstacles
        self.motion_model()    #8 motions
        self.calc_obstacle_map(ox, oy)  #position list of obstacles

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
                if len(visited_set.keys()) % 5 == 0:      # control the speed of animation by pauses
                    plt.pause(0.0001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break

            del empty_set[c_node]      # Remove the item from the empty set
            visited_set[c_node] = current   # Add it to the visited set

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion_model():
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_node)
                n_node = self.calc_index(node)

                if n_node in visited_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_node not in empty_set:
                    empty_set[n_node] = node  # Discover a new node

                else:
                    if empty_set[n_node].cost >= node.cost:
                        empty_set[n_node] = node    #update the node if find a lower cost

        rx, ry = self.calc_final_path(goal_node, visited_set)

        return rx, ry

    def calc_final_path(self, goal_node, visited_set):  #Generate path
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != -1:
            n = visited_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent
        return rx, ry

    def calc_position(self, index, minp):
        position = index * self.grid_size + minp
        return position

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.grid_size)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):   #check the model for obstacles
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
        self.obstacle_map = [[False for _ in range(0, 50)]
                             for _ in range(0, 75)]
        for ix in range(0, 75):
            x = self.calc_position(ix, self.min_x)
            for iy in range(0, 50):
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

    # start and goal position and radius and clearance
    sx = int(input("Enter the starting point x(Min allowed = 0): \n"))
    sy = int(input("Enter the starting point y(Min allowed = 0): \n"))
    gx = int(input("Enter the goal point x(Max allowed = 300): \n"))
    gy = int(input("Enter the goal point y(Max allowed = 200): \n"))
    radius = 2
    clearance = 2

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(False)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, radius, clearance)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()