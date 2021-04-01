# ENPM 661 - Planning for Autonomous Robots: 
# Project 3 Phase 2 - A* on Mobile Robot
# Shon Cortes, Bo-Shiang Wang

import matplotlib.pyplot as plt
import numpy as np

width = 400
height = 300


# Class for storing node position, cost to come, parent index, and prev_orientation.
class Node:
    def __init__(self, x, y, radius, cost, parent_index, prev_orientation):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.radius = radius
        self.prev_orientation = prev_orientation


def move_check(child_node):  # Check if the move is allowed.

    # Check if out of puzzle boundary
    if child_node.x < 0 or child_node.y < 0 or child_node.x >= width or child_node.y >= \
            height:
        return False

    # Check if obstacle
    elif obstacles_chk(child_node):
        return False

    else:
        return True


# Check if position is in Robot Adjusted obstacle space.
# Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15.
# Warning obstacles appear larger than they are.
def obstacles_chk(NODE):
    node = [NODE.x, NODE.y]
    # print(node)
    # Rectangle
    if (node[0] * 0.7) + 74.39 - 15 <= node[1] <= (node[0] * 0.7) + 98.568 + 15 \
            and (node[0] * -1.428) + 176.554 - 15 <= node[1] <= (node[0] * -1.428) + 438.068 + 15:
        return True

    # Circle
    elif (node[0] - 90) ** 2 + (node[1] - 70) ** 2 <= (35 + 15) ** 2:
        return True

    # Ellipse
    elif ((node[0] - 246) ** 2) / ((60 + 15) ** 2) + ((node[1] - 145) ** 2) / ((30 + 15) ** 2) <= 1:
        return True

    # 3 section Rectangular Area
    elif 200 - 15 <= node[0] <= 230 + 15 \
            and 230 - 15 <= node[1] <= 280 + 15:  # First section
        return True

    else:
        return False


def begin():  # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:

        start_x = int(input("Enter starting x coordinate: "))
        start_y = int(input("Enter starting y coordinate: "))
        try:
            start_theta = int(input("Enter starting theta: "))
        except ValueError:
            start_theta = 0

        # goal_x, goal_y = input("Enter goal x and y coordinates separated with a space: ").split()
        goal_x = int(input("Enter goal x coordinate: "))
        goal_y = int(input("Enter goal y coordinate: "))

        prev_orientation = start_theta
        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 15, 0, -1, prev_orientation)
        goal_node = Node(goal_x, goal_y, 15, 0, -1, 0)

        # Check if obstacle
        if obstacles_chk(start_node):
            print("Start position is in an obstacle.")
        elif obstacles_chk(goal_node):
            print("Goal position is in an obstacle.")

        # Check if values are positive and within the map
        elif start_node.x < 0 or start_node.y < 0 or start_node.x > width or start_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")
        elif goal_node.x < 0 or goal_node.y < 0 or goal_node.x > width or goal_node.y > \
                height:
            print("Please enter positive integer values (0 <= x <= 400, 0 <= y <= 300).")

        else:
            break

    return start_node, goal_node


def euclidean_dist(goal_node, node):
    dist = np.sqrt((goal_node.x - node.x)**2 + (goal_node.y - node.y)**2)
    return dist


def motion_model(orientation):
    orientation = np.deg2rad(orientation)
    theta = np.deg2rad(30)
    step_size = 10
    model = [[step_size * np.cos(2*theta + orientation), step_size * np.sin(2 * theta + orientation), 1, np.rad2deg(2 * theta + orientation)],  # 60
             [step_size * np.cos(theta + orientation), step_size * np.sin(theta + orientation), 1, np.rad2deg(theta + orientation)],  # 30
             [step_size * np.cos(orientation), step_size * np.sin(orientation), 1, np.rad2deg(orientation)],  # 0
             [step_size * np.cos(-theta + orientation), step_size * np.sin(-theta + orientation), 1, np.rad2deg(-theta + orientation)],  # -30
             [step_size * np.cos(-2 * theta + orientation), step_size * np.sin(-2 * theta + orientation), 1, np.rad2deg(-2 * theta + orientation)]  # -60
             ]

    return model


def a_star(start_node, goal_node):
    # Initialize dictionaries
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    queue[(start_node.x, start_node.y)] = start_node  # Initialize queue with startnode for Dijkstra algorithm.
    distance[(start_node.x, start_node.y)] = 0  # Initialize distance traveled.

    threshold = 0.5

    # Dictionary for orientation
    orientation_dict = {0: 0, 1: 30, 2: 60, 3: 90,
                        4: 120, 5: 150, 6: 180, 7: 210,
                        8: 240, 9: 270, 10: 300, 11: 330, 12: 0}

    # Create V matrix to store the information of the visited nodes.
    V = np.zeros((int(width/threshold), int(height/threshold), int(360/30)))

    while True:  # Start of A star Algorithm.
        # Find the node in queue with the minimum cost.
        cur_index = min(queue, key=lambda o: queue[o].cost + euclidean_dist(goal_node, queue[o]))
        # Assign node in queue with minimum cost to be the current node to be tested.
        cur = queue[cur_index]
        orientation = cur.prev_orientation

        # If goal node is reached, Break the while loop.
        # Add a threshold(circle) for the goal node
        if (goal_node.x-cur.x)**2 + (goal_node.y-cur.y)**2 <= 10**2:
            # print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index]  # Remove the current node from the queue.
        visited[cur_index] = cur  # Add current node to visited list.

        # Mark 1 for visited nodes in matrix V
        a = int(round(cur.x)/threshold)
        b = int(round(cur.y)/threshold)
        c = orientation_dict[orientation/30]
        V[a][b][c] = 1

        motion = motion_model(orientation)

        # Generate children of current node based on the action set.
        for i in range(len(motion)):
            next_x = round(cur.x + motion[i][0], 3)
            next_y = round(cur.y + motion[i][1], 3)
            child_orientation = round(motion[1][3])
            # Initialize action set

            # Generate child node
            node = Node(next_x, next_y, 15, cur.cost + motion[i][2], cur_index, orientation)
            # Assign child node position
            node_index = (node.x, node.y)

            if move_check(node):  # Check if child is within the map or in an obstacle.
                pass
            else:  # If out of bounds or an obstacle, restart loop and choose new node.
                continue

            a = int(round(node.x))
            b = int(round(node.y))
            c = node.prev_orientation
            # If the next node is already visited, skip it
            if V[a][b][c] == 1:
                continue
            # visualize_Dij(node_index)
            # Visualize motion
            # node_list.append((node.x, node.y))
            # print(node_list)
            plt.quiver(cur.x, cur.y, motion[i][0], motion[i][1], units='xy', scale=1, color='r', width = .05)
            plt.pause(.0001)
            # if node_index in visited:  # If the next node is already visited, skip it
            #     continue

            if node_index in queue:  # If the child node is already in the queue, compare and update the node's cost and parent as needed.
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:  # Else add child to the queue.
                queue[node_index] = node


    # # Backtrack the path from Goal to Start
    # path = []
    # parent_index = goal_node.parent_index
    # while parent_index != -1:  # Follow the parents from the goal node to the start node and add them to the path list.
    #     n = visited[parent_index]
    #     path.append((n.x, n.y))
    #     parent_index = n.parent_index

    # path = list(reversed(path))  # Reverse the path list to get the path from start node to goal node.
    # path.append((goal_node.x, goal_node.y))  # Add Goal node to the end of the path list
    
    # Backtrack the path from Goal to Start
    path_x, path_y = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    child = visited[parent_index]
    plt.quiver(child.x, child.y, goal_node.x, goal_node.y, units='xy', scale=1, color='r', width = .05)
    while parent_index != -1:  # Follow the parents from the goal node to the start node and add them to the path list.
        n = visited[parent_index]
        path_x.append(n.x)
        path_y.append(n.y)
        parent_index = n.parent_index

    # path = list(reversed(path))  # Reverse the path list to get the path from start node to goal node.
    # path.append((goal_node.x, goal_node.y))  # Add Goal node to the end of the path list
    # path_x.append(goal_node.x)
    # path_y.append(goal_node.y)
    return path_x, path_y

def main():

    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 400):
        for j in range(0, 300):
            # Circle
            if (i - 90) ** 2 + (j - 70) ** 2 <= 35 ** 2:
                ox.append(i)
                oy.append(j)

            # Ellipse
            if ((i - 246) ** 2) / (60 ** 2) + (((j - 145) ** 2) / (30 ** 2)) <= 1:
                ox.append(i)
                oy.append(j)

            if (i * 0.7) + 74.39 <= j <= (i * 0.7) + 98.568 \
                    and (i * -1.428) + 176.554 <= j <= (i * -1.428) + 438.068:
                ox.append(i)
                oy.append(j)

            # 3 Section Rectangular Area
            if 200 <= i <= 210 and 230 <= j <= 280:
                ox.append(i)
                oy.append(j)
            if 210 <= i <= 230 and 270 <= j <= 280:
                ox.append(i)
                oy.append(j)
            if 210 <= i <= 230 and 230 <= j <= 240:
                ox.append(i)
                oy.append(j)

    start_node, goal_node = begin()

    plt.xlim([0, 400])
    plt.ylim([0, 300])
    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")

    a = [start_node.x, start_node.y]
    b = [goal_node.x, goal_node.y]

    if a != b:
        path_x, path_y = a_star(start_node, goal_node)  # Call A star algorithm

        plt.plot(path_x, path_y, "-g")
        plt.pause(0.0001)
        plt.show()
    
    else:
        print("Start position equals the goal position.")

if __name__ == '__main__':
    main()