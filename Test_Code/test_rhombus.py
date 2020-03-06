
def nonconvex_obstacle_left_half(clearance, radius_rigid_robot, test_point_coord):

    augment_distance = radius_rigid_robot + clearance

    nonconvex_point_1 = [50,150]
    nonconvex_point_2 = [60,185]
    nonconvex_point_3 = [25,185]
    nonconvex_point_4 = [20,120]  
    
    #We set the flags by testing for a point inside the rectangle
    #Because the sign for the half plane is unique for every line, we test it by using a point that is confirmed to be inside the nonconvex_obstacle
    edge1_m_c = find_line_slope_and_intercept(test_point_coord,nonconvex_point_1, nonconvex_point_2)
    line1 = test_point_coord[1] - (edge1_m_c[0]*test_point_coord[0]) - (edge1_m_c[1] + (augment_distance*0))
    #print(line1)
    if line1>=0:
        flag1 = False
        print("False")
    else:
        flag1 = True
        print("True")
        
    
    edge2_m_c = find_line_slope_and_intercept(test_point_coord,nonconvex_point_2,nonconvex_point_3)
    line2 = test_point_coord[1] - (edge2_m_c[0]*test_point_coord[0]) - (edge2_m_c[1] + (augment_distance*1))  
    #print(line2)
    if line2>=0:
        flag2 = False
        print("False")
    else:
        flag2 = True
        print("True")
    
    #edge 3 is not augmented with clearance+robot_radius since its inside the nonconvex polygon
    edge3_m_c = find_line_slope_and_intercept(test_point_coord,nonconvex_point_3,nonconvex_point_4)
    line3 = test_point_coord[1] - (edge3_m_c[0]*test_point_coord[0]) - (edge3_m_c[1] + (augment_distance*0.0767))
    #print(line3)
    if line3>=0:
        flag3 = False
        print("False")
    else:
        flag3 = True
        print("True")
    
    
    edge4_m_c = find_line_slope_and_intercept(test_point_coord,nonconvex_point_4,nonconvex_point_5)
    line4 = test_point_coord[1] - (edge4_m_c[0]*test_point_coord[0]) - (edge4_m_c[1] - (augment_distance*0.7071))
    #print(line4)
    if line4>=0:
        flag4 = True
        print("True")
    else:
        flag4 = False
        print("False")
        
    if flag1 and flag2 and flag3 and flag4:
        return True
    else:
        return False
