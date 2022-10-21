from board import *
import copy
import heapq
from datetime import datetime
import sys

# Overwrite the default lt function.
def mylt(self, other):
    if self.f < other.f:
        return True
    elif self.f == other.f:
        if self.id < other.id:
            return True
        elif self.id > other.id:
            return False
        else:
            return self.parent.id < other.parent.id
    else:
        return False


State.__lt__ = mylt


def a_star(init_board, hfn):
    """
    Run the A_star search algorithm given an initial board and a heuristic function.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns am empty list and -1.

    :param init_board: The initial starting board.
    :type init_board: Board
    :param hfn: The heuristic function.
    :type hfn: Heuristic
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """
    h = hfn(init_board)
    init_state = State(init_board, hfn, h, 0, None)

    # frontier is a list of states
    frontier = []
    heapq.heappush(frontier, init_state)
    # explored is a list of state ids
    explored = []

    while len(frontier) != 0:

        nk = heapq.heappop(frontier)

        neighbors = get_successors(nk)
        for n in neighbors:
            
            n.f = hfn(n.board) + n.depth

        if nk.id not in explored:

            heapq.heappush(explored, nk.id)
            if is_goal(nk):
                curr_path = get_path(nk)
                # for p in curr_path: print(p.id)
                return curr_path, nk.depth
            for i in range(0, len(neighbors)):
                heapq.heappush(frontier, neighbors[i])

    empty = []
    return empty, -1


def dfs(init_board):
    """
    Run the DFS algorithm given an initial board.

    If the function finds a goal state, it returns a list of states representing
    the path from the initial state to the goal state in order and the cost of
    the solution found.
    Otherwise, it returns an empty list and -1.

    :param init_board: The initial board.
    :type init_board: Board
    :return: (the path to goal state, solution cost)
    :rtype: List[State], int
    """

    limit = 0
    init_state = State(init_board, zero_heuristic, 0, 0, None)

    # frontier is a list of paths, a path is a list of states
    frontier = [init_state]
    # explored is a list of state ids
    explored = set()
    cost = 0
    while len(frontier) != 0:
        # if limit == 100: break
        nk = frontier.pop(-1)

        neighbors = get_successors(nk)

        neighbors.sort(key=lambda x: x.id, reverse=True)

        if nk.id not in explored:

            explored.add(nk.id)
            
            if is_goal(nk):
                curr_path = get_path(nk)
                for p in curr_path: print(p.id)
                return curr_path, nk.depth
            for i in range(0, len(neighbors)):
                frontier.append(neighbors[i])
            cost = cost + 1
        limit += 1
    print(cost)
    empty = []
    return empty, -1


def copy_car(car):
    """
    Make a deepcopy of car
     
    :type car: Car
    :return: A copy of car
    :rtype: Car
    """
    if car is None: return None
    if car.orientation == "h":
        new_car = Car(car.var_coord, car.fix_coord,
                      car.orientation, car.length, car.is_goal)
    else:
        new_car = Car(car.fix_coord, car.var_coord,
                      car.orientation, car.length, car.is_goal)
    return new_car


def get_successors(state):
    """
    Return a list containing the successor states of the given state.
    The states in the list may be in any arbitrary order.

    :param state: The current state.
    :type state: State
    :return: The list of successor states.
    :rtype: List[State]
    """
    
    
    num_cars = len(state.board.cars)
    list_new_state = []

    # for each car in the board, 
    # there are two possible moves
    # the first possible move: move right or down        
    for i in range(num_cars):

        for j in range(1, state.board.size):

            # the candidates, initially the same as state

            car = copy_car(state.board.cars[i])

            # whether it is a valid successor
            valid = 0
            target_posn1 = car.var_coord + car.length
            target_posn2 = car.var_coord + car.length - 1 + j

            if target_posn2 <= state.board.size - 1 and target_posn1 >= 0:
                if car.orientation == "h":
                    valid = 1
                    car.var_coord += j
                    for l in range(target_posn1, target_posn2 + 1):
                        if state.board.grid[car.fix_coord][l] != ".":
                            valid = 0
                            car.var_coord -= j

                if car.orientation == "v":

                    valid = 1
                    car.var_coord += j
                    for l in range(target_posn1, target_posn2 + 1):
                        if state.board.grid[l][car.fix_coord] != ".":
                            valid = 0
                            car.var_coord -= j

            if valid == 0: break

            # we can update new_state and add it to
            # the successors

            list_cars = []

            for l in range(num_cars):
                list_cars.append(copy_car(state.board.cars[l]))

            if valid == 1:
                list_cars[i] = car

                new_board = Board(state.board.name, state.board.size, list_cars)


                new_depth = state.depth + 1

                new_f = state.hfn(new_board) + new_depth

                new_state = State(new_board, state.hfn, new_f, new_depth, state)

                heapq.heappush(list_new_state, new_state)

    # second possible move: right or up

    for i in range(num_cars):

        for j in range(1, state.board.size):

            # the candidates, initially the same as state

            car = copy_car(state.board.cars[i])

            # whether it is a valid successor
            valid = 0
            target_posn1 = car.var_coord - j
            target_posn2 = car.var_coord - 1

            if target_posn2 <= state.board.size - 1 and target_posn1 >= 0:
                if car.orientation == "h":
                    valid = 1
                    car.var_coord -= j
                    for l in range(target_posn1, target_posn2 + 1):
                        if state.board.grid[car.fix_coord][l] != ".":
                            valid = 0
                            car.var_coord += j

                if car.orientation == "v":
                    valid = 1
                    car.var_coord -= j
                    for l in range(target_posn1, target_posn2 + 1):
                        if state.board.grid[l][car.fix_coord] != ".":
                            valid = 0
                            car.var_coord += j

            if valid == 0: break

            # we can update new_state and add it to
            # the successors

            list_cars = []

            for l in range(num_cars):
                list_cars.append(copy_car(state.board.cars[l]))

            if valid == 1:
                list_cars[i] = car

                new_board = Board(state.board.name, state.board.size, list_cars)


                new_depth = state.depth + 1

                new_f = state.hfn(new_board) + new_depth

                new_state = State(new_board, state.hfn, new_f, new_depth, state)

                # if (state.parent == None or state.parent.id != new_state.id):

                heapq.heappush(list_new_state, new_state)

    return list_new_state


def is_goal(state):
    """
    Returns True if the state is the goal state and False otherwise.

    :param state: the current state.
    :type state: State
    :return: True or False
    :rtype: bool
    """

    board = state.board
    cars = board.cars
    for i in range(len(cars)):
        if cars[i].is_goal == True and cars[i].var_coord == state.board.size - cars[i].length:
            return True
    return False


def get_path(state):
    """
    Return a list of states containing the nodes on the path 
    from the initial state to the given state in order.

    :param state: The current state.
    :type state: State
    :return: The path.
    :rtype: List[State]
    """

    parent = state.parent
    path = [state]
    while parent:
        path = [parent, *path]
        parent = parent.parent
    return path




def blocking_heuristic(board):
    """
    Returns the heuristic value for the given board
    based on the Blocking Heuristic function.

    Blocking heuristic returns zero at any goal board,
    and returns one plus the number of cars directly
    blocking the goal car in all other states.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """
    
    
    cars = board.cars
    goal_car = None
    count = 1
    # if this is a goal board
    for i in range(len(cars)):
        if cars[i].is_goal:
            goal_car = cars[i]
            if cars[i].var_coord == 4:
                return 0
    for i in range(goal_car.var_coord + goal_car.length, board.size):
        if board.grid[goal_car.fix_coord][i] != '.':
            count += 1
    return count
    
    
    
    

def update(grid, x, y):
    """
    update grid based on the coordinates x and y
    """
    if grid[x][y] == "<":
        grid[x][y] = "."
        for i in range(1, len(grid) + 1):
            if grid[x][y + i] == "-": grid[x][y + i] = "."
            if grid[x][y + i] == ">":
                grid[x][y + i] = "."
                break
    if grid[x][y] == ">":
        grid[x][y] = "."
        for i in range(1, len(grid) + 1):
            if grid[x][y - i] == "-": grid[x][y - i] = "."
            if grid[x][y - i] == "<":
                grid[x][y - i] = "."
                break
    if grid[x][y] == "-":
        grid[x][y] = "."
        for i in range(1, len(grid) + 1):
            if grid[x][y + i] == "-": grid[x][y + i] = "."
            if grid[x][y + i] == ">":
                grid[x][y + i] = "."
                break
        for i in range(1, len(grid) + 1):
            if grid[x][y - i] == "-": grid[x][y - i] = "."
            if grid[x][y - i] == "<":
                grid[x][y - i] = "."
                break


def advanced_heuristic(board):
    """
    An advanced heuristic of your own choosing and invention.

    :param board: The current board.
    :type board: Board
    :return: The heuristic value.
    :rtype: int
    """
    grid = copy.deepcopy(board.grid)
    cars = board.cars
    goal_car = None
    count = 1
    # if this is a goal board
    for i in range(len(cars)):
        if cars[i].is_goal:
            goal_car = cars[i]
            if cars[i].var_coord == 4:
                return 0

    # find the index of those blocking cars
    blocking = []
    goal_posn = goal_car.fix_coord

    for i in range(len(board.cars)):
        temp_car = board.cars[i]
        if temp_car.orientation == "h": continue
        if temp_car.fix_coord < goal_car.var_coord: continue
        if temp_car.var_coord <= goal_posn <= temp_car.var_coord + temp_car.length - 1:
            blocking.append(i)

    total_cost = 1
    
    # check whether there are other cars block those blocking cars
    # if any, remove them
    for i in blocking:
        up_cost = 1
        down_cost = 1
        temp_car = board.cars[i]
        up = temp_car.length - (2 - temp_car.var_coord)
        down = 2 - temp_car.var_coord + 1

        for j in range(1, up + 1):

            if grid[temp_car.var_coord - j][temp_car.fix_coord] == ".":
                continue
            if (grid[temp_car.var_coord - j][temp_car.fix_coord] == "<" or
                    grid[temp_car.var_coord - j][temp_car.fix_coord] == "-" or
                    grid[temp_car.var_coord - j][temp_car.fix_coord] == ">"):
                up_cost += 1
                update(grid, temp_car.var_coord - j, temp_car.fix_coord)
            if (grid[temp_car.var_coord - j][temp_car.fix_coord] == "v" or
                    grid[temp_car.var_coord - j][temp_car.fix_coord] == "|" or
                    grid[temp_car.var_coord - j][temp_car.fix_coord] == "^"):
                break
        for j in range(1, down + 1):
            bottom_posn = temp_car.var_coord + temp_car.length - 1
            if grid[bottom_posn + j][temp_car.fix_coord] == ".":
                continue
            if (grid[bottom_posn + j][temp_car.fix_coord] == "<" or
                    grid[bottom_posn + j][temp_car.fix_coord] == "-" or
                    grid[bottom_posn + j][temp_car.fix_coord] == ">"):
                down_cost += 1
                update(grid, bottom_posn + j, temp_car.fix_coord)
            if (grid[bottom_posn + j][temp_car.fix_coord] == "v" or
                    grid[bottom_posn + j][temp_car.fix_coord] == "|" or
                    grid[bottom_posn + j][temp_car.fix_coord] == "^"):
                break
        total_cost += min(up_cost, down_cost)
    return total_cost

"""
limit = 0
for i in from_file("jams_posted.txt"): 
    
    start_time = datetime.now()
    print(a_star(i, blocking_heuristic)[1])
    end_time = datetime.now()
    print('Duration: {}'.format(end_time - start_time))
    limit+=1

"""
"""
start_time = datetime.now()
print(a_star(from_file("jams_posted.txt")[20], blocking_heuristic)[1])
end_time = datetime.now()
print('Duration: {}'.format(end_time - start_time))
"""

"""
sys.stdout = open("advanced.txt", "w")

j = 0
print("The results of my advanced heuristic")
for i in from_file("jams_posted.txt"): 
    print("test No.", j)
    j += 1
    print("number of states expanded:")
    x = a_star(i, advanced_heuristic)[1]
    print("cost:")
    print(x)
    
    

sys.stdout.close()
"""