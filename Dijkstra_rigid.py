# ENPM 661 - Planning for Autonomous Robots
# Project 2 - Implementing Djikstra's Algorithm for a rigid robot
# Team - Haixiang Fang          , UID - 116293242
#        Kulbir Singh Ahluwalia , UID - 116836050

#function returns false when the point is outside the circle
def circular_obstacle(clearance, radius_rigid_robot, test_point_coord):
	circle_center = (225,150)
	test_point_coord_x = test_point_coord[0]
	test_point_coord_y = test_point_coord[1]
	augment_distance = radius_rigid_robot + clearance

	distance_from_center = ((test_point_coord_x - circle_center[0])**2 + (test_point_coord_y - circle_center[1])**2)**0.5

	if distance_from_center > (25+augment_distance):
		return False
	else:
		return True


#function returns false when the point is outside the ellipse
def ellipsoid_obstacle(clearance, radius_rigid_robot, test_point_coord):
	ellipsoid_center = (150,100)
	test_point_coord_x = test_point_coord[0]
	test_point_coord_y = test_point_coord[1]
	augment_distance = radius_rigid_robot + clearance
	semi_major_axis = 40
	semi_minor_axis = 20

	distance_from_center = ((test_point_coord_x - ellipsoid_center[0])**2) / (semi_major_axis**2)

							+ (test_point_coord_y - ellipsoid_center[1])**2 / (semi_minor_axis**2)


	if distance_from_center > 1:
		return False
	else:
		return True

def square_obstacle(clearance, radius_rigid_robot, test_point_coord):

	circle_center = (225,150)
	augment_distance = radius_rigid_robot + clearance

	square_point_1 = [100,38.66025]
	square_point_2 = [35.0481,76.1603]
	square_point_3 = [30.0481,67.5]
	square_point_4 = [95,30]

	# edge1 = point_wrt_line(test_point_coord,square_point_2,square_point_1)
	# edge2 = point_wrt_line(test_point_coord,square_point_3,square_point_2)
	# edge3 = point_wrt_line(test_point_coord,square_point_4,square_point_3)
	# edge4 = point_wrt_line(test_point_coord,square_point_1,square_point_4)

	edge1 = point_wrt_line(test_point_coord,square_point_1,square_point_2)
	edge2 = point_wrt_line(test_point_coord,square_point_2,square_point_3)
	edge3 = point_wrt_line(test_point_coord,square_point_3,square_point_4)
	edge4 = point_wrt_line(test_point_coord,square_point_4,square_point_1)


	if (not(edge1 or edge2)) and (edge3 and edge4):
		return True

	else:
		return False



# def point_wrt_line(test_point_coord, line_point_1, line_point_2):
# 	a = line_point_2[1]-line_point_1[1]
# 	b = line_point_1[0]-line_point_2[0]
# 	c = (line_point_2[1]*line_point_1[0])-(line_point_2[0]*line_point_1[1])

# 	line = a*test_point_coord[0] + b*test_point_coord[1] + c

# 	if line>=0:
# 		return True #on the left of the line, or on the line

# 	else:
# 		return False #D<0, on right of the line


def point_wrt_line(test_point_coord, line_point_1, line_point_2):
	slope = (line_point_2[1]-line_point_1[1])/(line_point_2[0]-line_point_1[0])
	intercept = line_point_1[1]-(slope*line_point_1[0])
	#print(slope,intercept)

	line = test_point_coord[1] - (slope*test_point_coord[0]) - intercept
	if line>=0:
		return True #on the left of the line, or on the line

	else:
		return False #D<0, on right of the line






