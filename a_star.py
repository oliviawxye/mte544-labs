import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from enum import Enum

class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# This function return the path of the search

class Heuristic(Enum):
    MANHATTAN = 1
    EUCLIDEAN = 2

def calculate_heuristic(choice: Heuristic, start_point: tuple[int, int], goal_point: tuple[int, int]):
    """Calculates a heuristic cost between point a and b, based on the choice"""
    h = np.iinfo(np.int32).max # some arbitrarily large number
    row_dist = abs(goal_point[0]-start_point[0])
    col_dist = abs(goal_point[1]-start_point[1])
    if choice == Heuristic.MANHATTAN:
        h = row_dist + col_dist
    else:
        h = sqrt(row_dist*row_dist + col_dist*col_dist)
        
    return h

# <---- CHANGE THIS HERE ---->

CHOICE = Heuristic.MANHATTAN

# <---- CHANGE THIS HERE ---->

def return_path(current_node, maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path


def search(maze, start, end):
    maze = maze.copy().T

    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """
    # Get the shape of the maze
    no_rows, no_columns=np.shape(maze)

    # Check if the start and end are within the boundaries of the maze, and if they are not on a blocked path
    if (start[0] < 0 or start[0] >= no_rows or 
        start[1] < 0 or start[1] >= no_columns or 
        end[0] < 0 or end[0] >= no_rows or 
        end[1] < 0 or end[1] >= no_columns or
        maze[start[0], start[1]] > 0.8 or maze[end[0], end[1]] > 0.8):
        print("Start or end is on a wall, or outside the boundaries of the maze")
        return None
    
    # TODO PART 4 Create start and end node with initized values for g, h and f
    # Use None as parent if not defined
    start_node = Node(parent=None, position=start)
    start_node.g = 0     # cost from start Node
    start_node.h = calculate_heuristic(CHOICE, start_point=start, goal_point=end)  # heuristic estimated cost to end Node
    start_node.f = start_node.g + start_node.h

    end_node = Node(parent=None, position=end)
    end_node.g = calculate_heuristic(Heuristic.EUCLIDEAN, start_point=start, goal_point=end)
    end_node.h = 0       # heuristic estimated cost to end Node
    end_node.f = start_node.g+start_node.h
    print(f"start: {start_node.position}")
    print(f"end: {end_node.position}")

    # Initialize both yet_to_visit and visited dictionary
    # in this dict we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    yet_to_visit_dict = {}  # key is the position (tuple), value is the node
    # in this list we will put all node those already explored so that we don't explore it again
    # key is the position (tuple), value is True (boolean)
    visited_dict = {}

    # Add the start node
    yet_to_visit_dict[start_node.position] = start_node

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . serarch movement is left-right-top-bottom
    # (4 movements) from every positon

    move = [[-1, 0],  # go up
            [0, -1],  # go left
            [1, 0],  # go down
            [0, 1],
            [-1, 1],
            [-1, -1],
            [1, 1],
            [1, -1]]  # go right

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit dict and add this node to visited dict
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited dict then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit dict then ignore it
                d) else move the child to yet_to_visit dict
    """

    # Loop until you find the end

    while len(yet_to_visit_dict) > 0:

        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1

        # TODO: Get the current node with the lowest f value
        current_node = None
        current_fscore = None
        for position, node in yet_to_visit_dict.items():
            if current_fscore is None or node.f < current_fscore:
                current_fscore = node.f
                current_node = node

        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iterations:
            print("giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        # Pop current node out off yet_to_visit dict, add to visited list
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True

        # test if goal is reached or not, if yes then return the path
        if current_node.position == end_node.position:
            print("found the end")
            return return_path(current_node, maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move:

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # TODO PART 4 Make sure within range (check if within maze boundary)
            if (start[0] < 0 or start[0] >= no_rows or 
                start[1] < 0 or start[1] >= no_columns or 
                end[0] < 0 or end[0] >= no_rows or 
                end[1] < 0 or end[1] >= no_columns):
                continue

            # Make sure walkable terrain
            if maze[node_position[0], node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children

        for child in children:
            print(f"child: {child.position}")
            print(f"visited: {visited_dict}")

            # TODO PART 4 Child is on the visited dict (use get method to check if child is in visited dict, if not found then default value is False)
            if visited_dict.get(current_node.position) is not None:
                print("visited")
                continue

            # TODO PART 4 Create the f, g, and h values
            child.g = calculate_heuristic(Heuristic.EUCLIDEAN, child.position, start_node.position)
            # Heuristic costs calculated here, this is using eucledian distance
            child.h = calculate_heuristic(CHOICE, child.position, end_node.position)

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            child_node_in_yet_to_visit = yet_to_visit_dict.get(
                child.position, False)
            if (child_node_in_yet_to_visit is not False) and (child.g >= child_node_in_yet_to_visit.g):
                print("some random thing")
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_dict[child.position] = child
            print("to visit")
            