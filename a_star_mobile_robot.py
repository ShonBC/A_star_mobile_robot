import matplotlib.pyplot as plt
import numpy as np

show_animation = True

width = 400
height = 300

class Node: # Class for storing node position, cost to come, and parent index.
    def __init__(self, x, y, radius, cost, parent_index, theta = 30):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.radius = radius
        self.theta = theta


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

def obstacles_chk(
        NODE):  # Check if position is in Robot Adjusted obstacle space. Obstacle space was expanded by a radius of 10 + 5 for clearance for a total of 15. Warning obstacles appear larger than they are.

    node = [NODE.x, NODE.y]
    print(node)
    # Rectangle
    if node[1] >= (node[0] * 0.7) + 74.39 - 15 and node[1] <= (node[0] * 0.7) + 98.568 + 15 and node[1] >= (
            node[0] * -1.428) + 176.554 - 15 and node[1] <= (node[0] * -1.428) + 438.068 + 15:
        return True

    # Circle
    elif (node[0] - 90) ** 2 + (node[1] - 70) ** 2 <= (35 + 15) ** 2:
        return True

    # Ellipse
    elif ((node[0] - 246) ** 2) / ((60 + 15) ** 2) + ((node[1] - 145) ** 2) / ((30 + 15) ** 2) <= 1:
        return True

    # 3 section Rectangular Area
    elif node[0] >= 200 - 15 and node[0] <= 230 + 15 and node[1] >= 230 - 15 and node[1] <= 280 + 15:  # First section
        return True

    else:
        return False



def begin():  # Ask for user input of start and goal pos. Start and goal much be positive integers
    while True:

        start_x, start_y = input("Enter starting x and y coordinates separated with a space: ").split()
        goal_x, goal_y = input("Enter goal x and y coordinates separated with a space: ").split()
        start_x = int(start_x)
        start_y = int(start_y)
        goal_x = int(goal_x)
        goal_y = int(goal_y)

        # Initialize start and goal nodes from node class
        start_node = Node(start_x, start_y, 15, 0, -1)
        goal_node = Node(goal_x, goal_y, 15, 0, -1)

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


def motion_model():
    theta = 30*np.pi/180
    model = [[np.cos(2*theta), np.sin(2*theta), np.sqrt(np.cos(2*theta)**2+np.sin(2*theta)**2)],
             [np.cos(theta), np.sin(theta), np.sqrt(np.cos(theta)**2+np.sin(theta)**2)],
             [1, 0, 1],
             [np.cos(theta), -np.sin(theta), np.sqrt(np.cos(theta)**2+np.sin(theta)**2)],
             [np.cos(2*theta), -np.sin(2*theta), np.sqrt(np.cos(2*theta)**2+np.sin(2*theta)**2)]]

    return model


def a_star(start_node, goal_node):
    # Initialize dictionaries
    path, distance, queue, visited = dict(), dict(), dict(), dict()

    motion = motion_model()  # Initialize action set
    explored_map = []  # Initialize list of explored positions
    queue[(start_node.x, start_node.y)] = start_node  # Initialize queue with startnode for Dijkstra algorithm.
    distance[(start_node.x, start_node.y)] = 0  # Initialize distance traveled.

    threshold = [.5, 30]
    vis = []

    while True:  # Start of A star Algorithm.

        cur_index = min(queue, key=lambda o: queue[o].cost + euclidean_dist(goal_node, queue[
            o]))  # Find the node in queue with the minimum cost.
        cur = queue[cur_index]  # Assign node in queue with minimum cost to be the current node to be tested.


        # if cur.x == goal_node.x and cur.y == goal_node.y:  # If goal node is reached, Break the while loop.
        if (goal_node.x-threshold[0]) <= cur.x <= (goal_node.x+threshold[0]) \
                and (goal_node.y-threshold[0]) <= cur.y <= (goal_node.y+threshold[0]):
            # print("Goal!!!")
            goal_node.parent_index = cur.parent_index
            goal_node.cost = cur.cost
            break

        del queue[cur_index]  # Remove the current node from the queue.
        visited[cur_index] = cur  # Add current node to visited list.
        vis.append((cur.x, cur.y, cur.theta))

        explored_map.append((cur.x, cur.y))  # Add current node to explored map.

        for i in range(len(motion)):  # Generate childeren of current node based on the action set.

            node = Node(cur.x + round(motion[i][0], 4), cur.y + round(motion[i][1],4), 15, cur.cost + motion[i][2],
                        cur_index)  # Generate child node
            node_index = (node.x, node.y)  # Assign child node position

            if move_check(node):  # Check if child is within the map or in an obstacle.
                pass
            else:  # If out of bounds or an obstacle, restart loop and choose new node.
                continue

            # Visualize motion
            q1 = plt.quiver(cur.x, cur.y, node.x, node.y, units='xy', scale=1, color= 'r')

            if node_index in visited:  # If the next node is already visited, skip it
                continue

            if node_check(node, vis, threshold):
                continue

            if node_index in queue:  # If the child node is already in the queue, compare and update the node's cost and parent as needed.
                if queue[node_index].cost > node.cost:
                    queue[node_index].cost = node.cost
                    queue[node_index].parent_index = cur_index
            else:  # Else add child to the queue.
                queue[node_index] = node

    # Backtrack the path from Goal to Start
    path = []
    parent_index = goal_node.parent_index
    while parent_index != -1:  # Follow the parents from the goal node to the start node and add them to the path list.
        n = visited[parent_index]
        path.append((n.x, n.y))
        parent_index = n.parent_index

    path = list(reversed(path))  # Reverse the path list to get the path from start node to goal node.
    path.append((goal_node.x, goal_node.y))  # Add Goal node to the end of the path list


def node_check(child_node, vis, threshold):

    # visited_pos = vis
    for i in range(len(vis)):
        center = vis[i]
        if center[0] - threshold[0] <= child_node.x <= center[0] + threshold[0] and center[1] - threshold[0] <= child_node.y <= center[1] + threshold[0] and center[2] - threshold[1] <= child_node.theta <= threshold[1] + center[2]:
            return True
        else:
            return False

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

            if j >= (i * 0.7) + 74.39 and j <= (i * 0.7) + 98.568 and j >= (i * -1.428) + 176.554 and j <= (
                    i * -1.428) + 438.068:
                ox.append(i)
                oy.append(j)

            # 3 Section Rectangular Area
            if i >= 200 and i <= 210 and j >= 230 and j <= 280:
                ox.append(i)
                oy.append(j)
            if i >= 210 and i <= 230 and j >= 270 and j <= 280:
                ox.append(i)
                oy.append(j)
            if i >= 210 and i <= 230 and j >= 230 and j <= 240:
                ox.append(i)
                oy.append(j)

    start_node, goal_node = begin()
    a_star(start_node, goal_node)  # Call A star algorithm

    plt.xlim([0, 400])
    plt.ylim([0, 300])
    plt.grid(True)
    plt.axis("equal")
    plt.plot(ox, oy, ".k")
    

    
    # plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()


if __name__ == '__main__':
    main()

# EASY
# Display backtracking in differnt color
# Add User input for theta_start

# Difficult?
# Why does it take a long time?
# Ask about tracking robot orientation?
