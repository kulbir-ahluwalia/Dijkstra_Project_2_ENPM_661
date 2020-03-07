from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
import Obstacle_map_rigid as dr

show_animation = True
import sys

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
                               self.calc_xy_index(sy, self.min_y), 0, 0)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0, 0)

        empty_set = dict()
        visited_set = dict()

        empty_set[self.calc_index(start_node)] = start_node

        while 1:

            # print("The key with the smallest cost:", c_node)
            # print("The smallest cost:", empty_set[c_node])
            # print(empty_set)
            # if empty_set == None:
            #     print("Goal Can't be reached")
            #     break
            # show animation
            if show_animation:
                curr_node = min(empty_set, key=lambda c: empty_set[c].cost)  # the key whose value is the smallest
                current = empty_set[curr_node]
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), ".y")
                if len(visited_set.keys()) % 5 == 0:      # control the speed of animation by pauses
                    plt.pause(0.0001)


            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find Goal")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                break

            del empty_set[curr_node]      # Remove the item from the empty set
            visited_set[curr_node] = current   # Add it to the visited set

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion_model():
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, curr_node)
                new_node = self.calc_index(node)

                if new_node in visited_set:
                    continue

                if not self.verify_node(node):
                    continue

                if new_node not in empty_set:
                    empty_set[new_node] = node  # Discover a new node

                else:
                    if empty_set[new_node].cost >= node.cost:
                        empty_set[new_node] = node    #update the node if find a lower cost

        rx, ry = self.calc_final_path(goal_node, visited_set)

        return rx, ry

    def calc_final_path(self, goal_node, visited_set):  #Generate path
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != 0:
            n = visited_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent
        return rx, ry

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def calc_position(self, index, min_p):
        return index * self.grid_size + min_p

    def calc_xy_index(self, position, min_p):
        return (position - min_p) / self.grid_size


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
        # if self.obstacle_map[node.x][node.y]:
        #     return False
        # if self.obstacle_map[node.x][ node.y] == 1:
        else:
            return True

    def calc_obstacle_map(self, ox, oy):
        #self.obstacle_map = [[False for j in range(0, 50)]
                             # for j in range(0, 75)]
        self.obstacle_map = dr.obs()
        # a = np.copy(self.obstacle_map)
        # a[np.where(a==255)]=True
        # a[np.where(a==0)]=False
        plt.imshow(self.obstacle_map,cmap="gray")
        plt.show()
        # sys.exit(0)


        # for ix in range(0, 75):
        #     x = self.calc_position(ix, self.min_x)
        #     for iy in range(0, 50):
        #         y = self.calc_position(iy, self.min_y)
        #         for iox, ioy in zip(ox, oy):
        #             d = sqrt((iox - x) ** 2 + (ioy - y) ** 2)
        #             if d <= (self.radius + self.clearance):
        #                 self.obstacle_map[ix][iy] = True
        #                 break

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
    # for i in range(90, 110):
    #     ox.append(i)
    #     oy.append(40)
    # for i in range(90, 110):
    #     ox.append(i)
    #     oy.append(60)
    # for i in range(40, 60):
    #     ox.append(90)
    #     oy.append(i)
    # for i in range(40, 60):
    #     ox.append(110)
    #     oy.append(i)
    # def circle(x, y, r):
    #
    #     for i in range(20):
    #         jiao = float(i) / 20 * 2 * math.pi
    #         x1 = x + r * math.cos(jiao)
    #         y1 = y + r * math.sin(jiao)
    #         ox.append(x1)
    #         oy.append(y1)
    #






    # start and goal position and radius and clearance
    # sx = int(input("Enter the starting point x(Min allowed = 0): \n"))
    # sy = int(input("Enter the starting point y(Min allowed = 0): \n"))
    # gx = int(input("Enter the goal point x(Max allowed = 300): \n"))
    # gy = int(input("Enter the goal point y(Max allowed = 200): \n"))
    sx = 90
    sy = 90
    gx = 100
    gy = 100

    radius = 0
    clearance = 0


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