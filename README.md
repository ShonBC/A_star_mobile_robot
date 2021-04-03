# A_star_mobile_robot
Implamentation of A* algorithm on a mobile robot. The user inputs start and goal position as well as an angle the robot can turn.
Project 3 - Point Robot A* on Mobile Robot using Half-Planes for Obstacle Space

Shon Cortes, Bo-Shiang Wang

Packages Used:

	1. import numpy
    2. matplotlib.pyplot as plt
        - Used for visualization of the path planning.


Run the program:

	1. Run the program using your prefered method (terminal...)

    2. Program asks for user input for the start and goal coordinates. 
        X-coordinates must be between integer values 0 and 400.
        Y-coordinates must be between integer values 0 and 300.
        -Enter the x coordinates for start position.
        -Enter the y coordinates for start position.
        -Enter starting theta for initial orientation.
        -Enter the x coordinates for goal position.
        -Enter the y coordinates for goal position.
        -Enter the step size for the motion. Values of 10 to 20 work best to speed up search.


    Example:
    -Enter the x coordinates for start position: 0
    -Enter the y coordinates for start position: 0
    -Enter starting theta for initial orientation: 90
    -Enter the x coordinates for goal position: 400
    -Enter the y coordinates for goal position: 300
    -Enter the step size for the motion: 20
    
Program Summary:
    After user provides input for start coordinates, goal coordinates, initial orientaiton, and step size an A* algorithm explores the map to find a path to the goal.
    The program opens a window that shows the map being explored. The white space represents the open map area, any black space represents an obstacel. 
    As the program runs, the explored space plots the node vectors in red. Once a path to the goal is found, the path is highlighted in green.