# Instructions on Running the code
1. Dijkstra_point.py:
Run Dijkstra_point.py. 
Enter the starting point and the goal point in the console panel. 
If either of them is inside of the obstacles or outside of the boundry(300x200), the program will return the error. 
If inputs are correct, the program will start seaching from bottom left after it finds the solution and drwa the optimal path after reaching the goal point. 


2. Dijkstra_rigid.py:
Run Dijkstra_rigid.py. 
The robot radius and clearence are fixed as 2 and 2. Enter the starting point and goal point in the console panel. 
If they're inside of the obstacles or outside of the boundry(300x200), the program will return the error. 
If inputs are correct, the program will start seaching from bottom left after it finds the solution and drwa the optimal path after reaching the goal point. 


1. Change the directory to the Code folder containing Dijkstra_rigid.py
2. Type "python3 Dijkstra_rigid.py" in the terminal
3. Enter the desired radius, clearance, start position and goal position.
4. For example, the start and goal position is input as [5,5] and [295,195].
5. Wait for 40 seconds and then the animation will pop up. The cost and time to completion will also be printed.

## Libraries used in the code
1. **cv2**
2. **numpy**
3. **math**
4. **matplotlib**
5. **time**

The test code folder contains jupyter notebooks used in the making of the code.
Code is commented where needed.



















