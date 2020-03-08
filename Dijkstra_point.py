<<<<<<< HEAD
# ENPM 661 - Planning for Autonomous Robots
# Project 2 - Implementing Djikstra's Algorithm for imag rigid robot
# Team - Haixiang Fang          , UID - 116293242
#        Kulbir Singh Ahluwalia , UID - 116836050
import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
from time import process_time


def cart2img(adjust_coord):
    return [adjust_coord[0], 200 - adjust_coord[1]]


def find_line_slope_and_intercept(test_point_coord, line_point_1, line_point_2):
    slope = (line_point_2[1] - line_point_1[1]) / (line_point_2[0] - line_point_1[0])
    intercept = line_point_1[1] - (slope * line_point_1[0])
    # print(slope,intercept)
    return slope, intercept


# function returns false when the point is outside the circle
def circular_obstacle(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (225, 150)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance

    distance_from_center = ((test_point_coord_x - circle_center[0]) ** 2 + (
                test_point_coord_y - circle_center[1]) ** 2) ** 0.5

    if distance_from_center > (25 + augment_distance):
        return False
    else:
        return True


# function returns false when the point is outside the ellipse
def ellipsoid_obstacle(clearance, radius_rigid_robot, test_point_coord):
    ellipsoid_center = (150, 100)
    test_point_coord_x = test_point_coord[0]
    test_point_coord_y = test_point_coord[1]
    augment_distance = radius_rigid_robot + clearance
    semi_major_axis = 40
    semi_minor_axis = 20

    distance_from_center = ((test_point_coord_x - ellipsoid_center[0]) ** 2) / (
                (semi_major_axis + augment_distance) ** 2) + (test_point_coord_y - ellipsoid_center[1]) ** 2 / (
                                       (semi_minor_axis + augment_distance) ** 2)

    if distance_from_center > 1:
        return False
    else:
        return True


def rectangle_obstacle(clearance, radius_rigid_robot, test_point_coord):
    circle_center = (225, 150)
    augment_distance = radius_rigid_robot + clearance

    rectangle_point_1 = [100, 38.66025]
    rectangle_point_2 = [35.0481, 76.1603]
    rectangle_point_3 = [30.0481, 67.5]
    rectangle_point_4 = [95, 30]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_1, rectangle_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (
                edge1_m_c[1] + (augment_distance * 2 / (3 ** 0.5)))
    # print(line1)
    if line1 >= 0:
        flag1 = False
        # print("False")
    else:
        flag1 = True
        # print("True")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_2, rectangle_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance * 2))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    edge3_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_3, rectangle_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (
                edge3_m_c[1] - (augment_distance * 2 / (3 ** 0.5)))
    # print(line3)
    if line3 >= 0:
        flag3 = True
        # print("True")
    else:
        flag3 = False
        # print("False")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, rectangle_point_4, rectangle_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance * 2))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def rhombus_obstacle(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    rhombus_point_1 = [250, 25]
    rhombus_point_2 = [225, 40]
    rhombus_point_3 = [200, 25]
    rhombus_point_4 = [225, 10]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the rectangle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_1, rhombus_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] + (augment_distance / 0.8575))
    # print(line1)
    if line1 >= 0:
        flag1 = False
    else:
        flag1 = True

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_2, rhombus_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 0.8575))
    # print(line2)
    if line2 >= 0:
        flag2 = False
    else:
        flag2 = True

    edge3_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_3, rhombus_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] - (augment_distance / 0.8575))
    # print(line3)
    if line3 >= 0:
        flag3 = True
    else:
        flag3 = False

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, rhombus_point_4, rhombus_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.8575))
    # print(line4)
    if line4 >= 0:
        flag4 = True
    else:
        flag4 = False

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def nonconvex_obstacle_right_half(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [100, 150]
    nonconvex_point_2 = [75, 185]
    nonconvex_point_3 = [60, 185]
    nonconvex_point_4 = [50, 150]
    nonconvex_point_5 = [75, 120]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_1, nonconvex_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] + (augment_distance / 0.58124))
    # print(line1)
    if line1 >= 0:
        flag1 = False
        # print("False")
    else:
        flag1 = True
        # print("True")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_2, nonconvex_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 1))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_3, nonconvex_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] + (augment_distance / 0.27472))
    # print(line3)
    if line3 >= 0:
        flag3 = False
        # print("False")
    else:
        flag3 = True
        # print("True")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_4, nonconvex_point_5)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.64018))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    edge5_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_5, nonconvex_point_1)
    line5 = test_point_coord[1] - (edge5_m_c[0] * test_point_coord[0]) - (edge5_m_c[1] - (augment_distance / 0.640184))
    # print(line4)
    if line5 >= 0:
        flag5 = True
        # print("True")
    else:
        flag5 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4 and flag5:
        return True
    else:
        return False


def nonconvex_obstacle_left_half(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [50, 150]
    nonconvex_point_2 = [60, 185]
    nonconvex_point_3 = [25, 185]
    nonconvex_point_4 = [20, 120]

    # We set the flags by testing for image point inside the rectangle
    # Because the sign for the half plane is unique for every line, we test it by using image point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_1, nonconvex_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0] * test_point_coord[0]) - (edge1_m_c[1] - (augment_distance / 0.27472))
    # print(line1)
    if line1 >= 0:
        flag1 = True
        # print("True")
    else:
        flag1 = False
        # print("False")

    edge2_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_2, nonconvex_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0] * test_point_coord[0]) - (edge2_m_c[1] + (augment_distance / 1))
    # print(line2)
    if line2 >= 0:
        flag2 = False
        # print("False")
    else:
        flag2 = True
        # print("True")

    # edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_3, nonconvex_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0] * test_point_coord[0]) - (edge3_m_c[1] + (augment_distance / 0.0767))
    # print(line3)
    if line3 >= 0:
        flag3 = False
        # print("False")
    else:
        flag3 = True
        # print("True")

    edge4_m_c = find_line_slope_and_intercept(test_point_coord, nonconvex_point_4, nonconvex_point_1)
    line4 = test_point_coord[1] - (edge4_m_c[0] * test_point_coord[0]) - (edge4_m_c[1] - (augment_distance / 0.7071))
    # print(line4)
    if line4 >= 0:
        flag4 = True
        # print("True")
    else:
        flag4 = False
        # print("False")

    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False


def boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
    augment_distance = radius_rigid_robot + clearance
    x = test_point_coord[0]
    y = test_point_coord[1]

    if 0 <= x < augment_distance:
        return True
    elif (299 - augment_distance) < x <= 299:
        return True
    elif 0 <= y < augment_distance:
        return True
    elif (199 - augment_distance) < y <= 199:
        return True
    else:
        return False


class GraphNode:
    def __init__(self, point):
        self.position = point
        self.cost = math.inf
        self.parent = None


def test_point_obstacle_check(clearance, radius_rigid_robot, test_point_coord, image):
    test_point_coord = cart2img(test_point_coord)
    if circular_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif ellipsoid_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif rectangle_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif rhombus_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif nonconvex_obstacle_right_half(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif nonconvex_obstacle_left_half(clearance, radius_rigid_robot, test_point_coord):
        return True
    elif boundary_obstacle(clearance, radius_rigid_robot, test_point_coord):
        return True
    else:
        return False


def action_up(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 1
    up_point_coord = [test_point_coord[0], test_point_coord[1] - 1]
    if test_point_coord[1] > 0 and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, up_point_coord, image)) == False):
        # up_point_coord = [test_point_coord[0],test_point_coord[1]-1]
        return up_point_coord, action_cost
    else:
        return None, None


def action_down(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 1
    down_point_coord = [test_point_coord[0], test_point_coord[1] + 1]
    if test_point_coord[1] < 199 and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, down_point_coord, image)) == False):
        # down_point_coord = [test_point_coord[0],test_point_coord[1]+1]
        return down_point_coord, action_cost
    else:
        return None, None,


def action_left(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 1
    left_point_coord = [test_point_coord[0] - 1, test_point_coord[1]]
    if test_point_coord[0] > 0 and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, left_point_coord, image)) == False):
        # left_point_coord = [test_point_coord[0]-1,test_point_coord[1]]
        return left_point_coord, action_cost
    else:
        return None, None


def action_right(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 1
    right_point_coord = [test_point_coord[0] + 1, test_point_coord[1]]
    if test_point_coord[0] < 299 and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, right_point_coord, image)) == False):
        # right_point_coord = [test_point_coord[0]+1,test_point_coord[1]]
        return right_point_coord, action_cost
    else:
        return None, None


def action_up_right(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 2 ** 0.5
    up_right_point_coord = [test_point_coord[0] + 1, test_point_coord[1] - 1]
    if (test_point_coord[1] > 0 and test_point_coord[0] < 299) and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, up_right_point_coord, image)) == False):
        # up_right_point_coord = [test_point_coord[0]+1,test_point_coord[1]-1]
        return up_right_point_coord, action_cost
    else:
        return None, None


def action_down_right(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 2 ** 0.5
    down_right_point_coord = [test_point_coord[0] + 1, test_point_coord[1] + 1]
    if (test_point_coord[1] < 199 and test_point_coord[0] < 299) and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, down_right_point_coord, image)) == False):
        return down_right_point_coord, action_cost
    else:
        return None, None


def action_up_left(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 2 ** 0.5
    up_left_point_coord = [test_point_coord[0] - 1, test_point_coord[1] - 1]
    if (test_point_coord[1] > 0 and test_point_coord[0] > 0) and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, up_left_point_coord, image)) == False):
        return up_left_point_coord, action_cost
    else:
        return None, None


def action_down_left(image, clearance, radius_rigid_robot, test_point_coord):
    action_cost = 2 ** 0.5
    down_left_point_coord = [test_point_coord[0] - 1, test_point_coord[1] + 1]
    if (test_point_coord[1] < 199 and test_point_coord[0] > 0) and (
            (test_point_obstacle_check(clearance, radius_rigid_robot, down_left_point_coord, image)) == False):
        # down_left_point_coord = [test_point_coord[0]-1,test_point_coord[1]+1]
        return down_left_point_coord, action_cost
    else:
        return None, None


def get_new_node(image, action, clearance, radius_rigid_robot, test_point_coord):
    action_map = {
        'U': action_up(image, clearance, radius_rigid_robot, test_point_coord),
        'D': action_down(image, clearance, radius_rigid_robot, test_point_coord),
        'L': action_left(image, clearance, radius_rigid_robot, test_point_coord),
        'R': action_right(image, clearance, radius_rigid_robot, test_point_coord),
        'UR': action_up_right(image, clearance, radius_rigid_robot, test_point_coord),
        'UL': action_up_left(image, clearance, radius_rigid_robot, test_point_coord),
        'DR': action_down_right(image, clearance, radius_rigid_robot, test_point_coord),
        'DL': action_down_left(image, clearance, radius_rigid_robot, test_point_coord)

    }
    return action_map[action]


def get_minimum_element(queue):
    min_index = 0
    for index in range(len(queue)):
        if queue[index].cost < queue[min_index].cost:
            min_index = index
    return queue.pop(min_index)


def find_path_dijkstra(image, start_node_pos, goal_node_pos, clearance, radius_rigid_robot):
    start_node = GraphNode(start_node_pos)
    start_node.cost = 0

    visited = list()
    queue = [start_node]

    actions = ["U", "D", "L", "R", "UR", "DR", "UL", "DL"]
    visited_set = set()
    visited_list = []
    cost_updates_matrix = np.zeros((200, 300), dtype=object)

    cost_updates_matrix[:, :] = math.inf
    goal_reached = False
    parent_child_map = {}
    parent_child_map[tuple(start_node_pos)] = None

    start = process_time()
    while len(queue) > 0:
        current_node = get_minimum_element(queue)
        current_point = current_node.position
        visited.append(str(current_point))
        visited_set.add(str(current_point))
        visited_list.append(current_point)

        if current_point == goal_node_pos:
            goal_reached = True
            print("Cost = ", current_node.cost)
            break

        child_nodes = []
        for action in actions:
            new_point, base_cost = get_new_node(image, action, clearance, radius_rigid_robot, current_point)
            if new_point is not None:
                child_nodes.append((new_point, base_cost))

        for child in child_nodes:
            if str(child[0]) not in visited_set:
                prev_cost = cost_updates_matrix[child[0][1], child[0][0]]
                new_cost = child[1] + current_node.cost
                if new_cost < prev_cost:
                    cost_updates_matrix[child[0][1], child[0][0]] = new_cost
                    child_node = GraphNode(child[0])
                    child_node.cost = new_cost
                    child_node.parent = current_node
                    queue.append(child_node)
                    parent_child_map[tuple(child[0])] = tuple(current_point)

    end = process_time()
    print("Time to completion:", (end - start))

    if goal_reached:
        print('Reached')
        return visited_list, parent_child_map
    else:
        return None, None


def check_inputs_wrt_obstacles(start_node_x, start_node_y, goal_node_x, goal_node_y):
    if test_point_obstacle_check(clearance, radius_rigid_robot, [start_node_x, start_node_y], None):
        print("Start node is inside an obstacle. Enter some other coordinates. Restart program!")
        exit(0)

    if test_point_obstacle_check(clearance, radius_rigid_robot, [goal_node_x, goal_node_y], None):
        print("Goal node is inside an obstacle. Enter some other coordinates. Restart program!")
        exit(0)


radius_rigid_robot = 0
clearance = 0
# Uncomment to choose different positions:-
start_node_x = int(input("Enter the starting x coordinate for the rigid robot\n"))
start_node_y = int(input("Enter the starting y coordinate for the rigid robot\n"))
goal_node_x = int(input("Enter the goal x coordinate for the rigid robot\n"))
goal_node_y = int(input("Enter the goal y coordinate for the rigid robot\n"))

start_node_y = 200 - start_node_y
goal_node_y = 200 - goal_node_y
# for testing
# start_node_x = 5
# start_node_y = 10
# goal_node_x = 20
# goal_node_y = 30
# clearance = 2
# radius_rigid_robot = 2


if (start_node_x < 0 and start_node_x > 300) and (goal_node_x < 0 and goal_node_x > 300):
    print("X coordinate is out of range. Enter x from [0,300]. Restart program!")
    exit(0)

if (start_node_y < 0 and start_node_y > 200) and (goal_node_y < 0 and goal_node_y > 200):
    print("Y coordinate is out of range. Enter y from [0,200]. Restart program!")
    exit(0)

check_inputs_wrt_obstacles(start_node_x, start_node_y, goal_node_x, goal_node_y)


# print(test_point_obstacle_check(0,0,[230,40]))


def plot_map(clearance, radius_rigid_robot):
    image = np.ones((200, 300, 3), np.uint8) * 255

    # print("Circle: ", circular_obstacle(r, c, [225, 150]))

    for i in range(0, 300):
        for j in range(0, 200):
            # print("For Loop")
            idx = cart2img([i, j])
            # print("Circle: ", circular_obstacle(r, c, [225, 150]))
            if circular_obstacle(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ",i,j)
                image[j, i] = (0, 0, 0)

            if ellipsoid_obstacle(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ",i,j)
                image[j, i] = (0, 0, 0)

            if rhombus_obstacle(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ",i,j)
                image[j, i] = (0, 0, 0)

            if rectangle_obstacle(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ",i,j)
                image[j, i] = (0, 0, 0)

            if nonconvex_obstacle_right_half(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ",i,j)
                image[j, i] = (0, 0, 0)

            if nonconvex_obstacle_left_half(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ", i, j)
                image[j, i] = (0, 0, 0)

            if boundary_obstacle(radius_rigid_robot, clearance, [idx[0], idx[1]]) == True:
                # print("Circle: ", i, j)
                image[j, i] = (0, 0, 0)
            # image[np.where(image==255)]=True
            # image[np.where(image==0)]=False
    return image


def main():
    image = plot_map(clearance, radius_rigid_robot)

    adjusted_coord_start_node = ([start_node_x, start_node_y])
    adjusted_coord_goal_node = ([goal_node_x, goal_node_y])

    image[adjusted_coord_start_node[1], adjusted_coord_start_node[0]] = (0, 255, 10)
    image[adjusted_coord_goal_node[1], adjusted_coord_goal_node[0]] = (10, 0, 255)

    start_node_position = [start_node_x, start_node_y]
    goal_node_position = [goal_node_x, goal_node_y]

    visited_list, parent_child_map = find_path_dijkstra(image, start_node_position, goal_node_position, clearance,
                                                        radius_rigid_robot)

    for v in visited_list:
        image[v[1], v[0]] = (255, 255, 0)
        resized_new = cv2.resize(image, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)
        cv2.imshow("Figure", resized_new)
        if cv2.waitKey(1) == 27:
            break

    trace_path = []
    parent = parent_child_map[tuple(goal_node_position)]
    while parent is not None:
        trace_path.append(parent)
        parent = parent_child_map[parent]

    trace_path.reverse()
    for point in trace_path:
        image[point[1], point[0]] = (200, 0, 200)
        resized_new = cv2.resize(image, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)

    cv2.imshow("Figure", resized_new)

    print("Press any key to Quit")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    plt.imshow(image)
    print(image)
    plt.show()


if __name__ == "__main__":
    main()


=======
# ENPM 661 - Planning for Autonomous Robots
# Project 2 - Implementing Djikstra's Algorithm for a rigid robot
# Team - Haixiang Fang          , UID - 116293242
#        Kulbir Singh Ahluwalia , UID - 116836050


import matplotlib.pyplot as plt
import math
from math import sqrt
import sys
import Obstacle_map_rigid
show_animation = True
from time import process_time
import numpy as np
import matplotlib.pyplot as plt

max_y = 200
max_x = 300

################################################################
class Dijkstra:
    def __init__(self, ox, oy, radius, clearance):
        self.min_x = 0
        self.min_y = 0
        self.max_x = max_x
        self.max_y = max_y
        self.grid_size= 4
        self.x_width = 300/self.grid_size
        self.y_width = 200/self.grid_size
        self.obstacle_map = None

        self.radius = radius   # robot radius
        self.clearance = clearance  # clearance for obstacles
        self.motion_model()    # 8 motions
        self.calc_obstacle_map(ox, oy)  # position list of obstacles

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
        start = process_time()
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
        end = process_time()
        print("Time to completion:", (end-start))

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
        return index * self.grid_size + minp

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
        else:
            return True

    def calc_obstacle_map(self, ox, oy):
        self.obstacle_map = [[False for j in range(0, 50)]
                             for j in range(0, 75)]
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
    map = Obstacle_map_rigid.obs()
    [ox, oy] = np.where(map == 1)
    X = np.array([ox, oy])
    X = np.array([[0, 1], [-1, 0]]).dot(X)  #Rotate 90 degree
    [ox, oy] = X
    oy = oy + max_y

    # start and goal position and radius and clearance
    while 1:
        sx = int(input("Enter the starting point x(Min allowed = 0): \n"))
        sy = int(input("Enter the starting point y(Min allowed = 0): \n"))
        gx = int(input("Enter the goal point x(Max allowed = 300): \n"))
        gy = int(input("Enter the goal point y(Max allowed = 200): \n"))
        if sx >300 or sx < 0:
            print("Start node invalid")
            continue
        if sy >200 or sy < 0:
            print("Goal node invalid")
            continue
        if map[sy,sx] == 1:
            print("The start point can not be inside the obstacle")
            continue
        if map[gy,gx] == 1:
            print("The goal point can not be inside the obstacle")
            continue
        else:
            break
    radius = 0
    clearance = 0

    if show_animation:

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
>>>>>>> a808d503847b2e829f342a4eeef5087fbb8f715a
