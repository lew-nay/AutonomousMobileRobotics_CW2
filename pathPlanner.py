# Import any libraries required

class Node:
    """Class for creating nodes based on a given parent node's position and own position in 2D cartesian co-ordinates
    
    Parameters:
    - parent = tuple of (x,y) co-ordinates
    - position = tuple of (x,y) co-ordinates
    """
    def __init__(self, parent=None, position=None):
        # parent node's position
        self.parent = parent
        # own position
        self.position = position

        # declaring variables
        # actual cost from start node
        self.g = 0 
        # estimated cost (heuristic)
        self.h = 0 
        # sum of g and h
        self.f = 0 

def heuristic(current, end):
    """calculates heuristic using Euclidian distance takes current position and end goal position
    
    Parameters:
    - current = an instance of the Node class, the current node
    - end = an instance of the Node class, the end goal node
    """
    return ((end.position[0] - current.position[0])**2 + (end.position[1] - current.position[1])**2)**0.5

def get_neighbours(current, grid, COL, ROW):
    """returns all valid neighbouring nodes given an input node
    
    Parameters:
    - current = an instance of the Node class, the current node
    - grid = 2D list of 1s and 0s, 1 representing open space, 0 representing blocked space
    - COL = integer value, the width of the grid
    - ROW = integer value, the height of the grid

    Returns:
    - neighbours = a list of the position of all valid neighbouring nodes
    """

    # list to store position of neighbour nodes as tuples
    neighbours = []

    # for use in checking each legal direction: right, left, up, down
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    # loops through each neighbour's position and checks if it is a valid node to move towards
    for direction in directions:
        neighbour_position = (current.position[0] + direction[0], current.position[1] + direction[1])

        # check these neighbour co-ords are valid (within grid bounds)
        if 0 <= neighbour_position[0] < COL and 0 <= neighbour_position[1] < ROW:
             # checks for if the neighbour is an obxtacle or not
             # if not an obstacle, appends to neighbours to be returned
            if grid[neighbour_position[0]][neighbour_position[1]] == 1:
                neighbours.append(Node(position=neighbour_position, parent=current))
    
    return neighbours


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    """
    Function for performing the A* path finding algorithm.

    Parameters: 
    - grid = a 2D list of 1's and 0's, 1's representing open space, 0's reprenting occupied space
    - start = a tuple of (x,y) co-ordinates indicating the starting node's location
    - end = a tuple of (x,y) co-ordinates indicating the goal node's location
    - display_message = a function used for displaying debugging messages
    """
    #EDIT ANYTHING BELOW HERE
    
    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])

    # nodes yet to be explored
    open_list = [] 
    # nodes that have been explored
    # type set used as lookups in sets compute at O(1)
    closed_list = set()
    # final path
    path = [] 

    # create 2 node instances from given start and end points for
    # later use
    start_node = Node(position=start)
    end_node = Node(position=end)

    # early return in the case of start == end to save computation
    if start_node.position == end_node.position:
        return [start_node.position]

    # set up values for the start node, g defaults to 0
    start_node.h = heuristic(start_node, end_node)
    start_node.f = start_node.g + start_node.h

    # add starting node to the list
    open_list.append(start_node)

    # loop till a path is found
    while open_list:
        # sort the list based on value of f
        open_list.sort(key=lambda node: node.f) 
        # get the node with lowest f value
        current_node = open_list.pop(0)
        # we have now visited this node add it to the closed list
        closed_list.add(current_node.position) 
                                         
        # reconstruct the path now that the end goal has been reached
        # this while loop appends each node's position to path until 
        # there are no more nodes remaining, i.e. the path has been fully reconstructed
        if current_node.position == end_node.position:
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent

            # as we want start to finish, not finish to start
            path.reverse() 
            return path

        # check neighbouring nodes's costs, if there is a lower cost node, ignore the current node
        # if the current node is the new lowest cost, that node gets added to the open list
        for neighbour in get_neighbours(current_node, grid, COL, ROW):
            if neighbour.position in closed_list:
                continue # skip already visited nodes
            
            # +1 as moving one square
            neighbour.g = current_node.g + 1 
            # calculate the heursitic value for this neightbour node to the end node
            neighbour.h = heuristic(neighbour, end_node)
            # add this g and h together to get total cost of neighbour node
            neighbour.f = neighbour.g + neighbour.h

            # flag used for comparion logic of lowest cost nodes
            found_better: bool = False

            # Check if there is a node in open_list with a lower cost at the same position
            for node in open_list:
                if node.position == neighbour.position and node.g <= neighbour.g:
                    found_better = True
                    # Skip adding this node
                    break 
            if not found_better:
                # Only add if it's not already in open_list with a lower cost
                open_list.append(neighbour)  
    
    # Print an example debug message
    display_message("Start location is " + str(start))


    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES

    # if no path found, returns an empty list
    return []
#end of file