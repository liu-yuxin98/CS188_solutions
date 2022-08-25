# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()



def get_actions(start_state,goal_state,parent_dict):
    # given start_state and goal_state, return action lst from start_state to goal_state
    from game import Directions
    from game import Actions
    path = []
    while True:
        if goal_state == start_state:
            path.append(goal_state)
            break
        path.append(goal_state)
        goal_state = parent_dict[goal_state][0]
    path = path[::-1]
    action_lst = [ Actions.vectorToDirection( (path[i+1][0]-path[i][0],path[i+1][1]-path[i][1]) ) for i in range(0,len(path)-1)]
    return action_lst

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # print('self.startState',problem.startState)

    # print('problem.walls', problem.walls)
    # print('problem.goal',problem.goal)
    # print('problem.costFn',problem.costFn)
    # print('problem.visualize',problem.visualize)
    # print('problem._visited',problem._visited)
    # print('problem._visitedlist',problem._visitedlist)
    # print('problem._expanded',problem._expanded)
    from util import Stack
    fringe = Stack()
    
    current = (problem.getStartState(), [], [])
    fringe.push(current)
    reached = []
        
    while not fringe.isEmpty():
        node, path, total = fringe.pop()
        if problem.isGoalState(node):
            return path
        if not node in reached:
            reached.append(node)
            for coord, move, cost in problem.getSuccessors(node):
                fringe.push((coord, path + [move], total + [cost])) 
    return []
    # from util import Stack
    # from game import Actions

    # parent_dict = {}  # record each state's predecessor
    # reached = set()
    # fringe = Stack()

    # # if problem.IS-GOAL(node.STATE) then return node
    # if problem.isGoalState(problem.getStartState()):
    #     return []

    # # frontier ←a LIFO queue (stack) with node as element
    # # reached ← {problem.INITIAL-STATE}    
    # fringe.push(problem.getStartState())
    # reached.add(problem.getStartState())
    # parent_dict[problem.getStartState()] = 'Start'

    # #  while not IS-EMPTY(frontier) do
    # while not fringe.isEmpty():
    #     # node ← POP(frontier)
    #     current_state = fringe.pop()
    #     reached.add(current_state)
    #     # for each child in EXPAND(problem, node) do
    #     succssor_lst = problem.getSuccessors(current_state)
    #     for succssor in succssor_lst: 
    #         child_state = succssor[0]
    #         parent_to_child_action = succssor[1]
    #         parent_to_child_cost = succssor[2]

    #         # ignore already reached child
    #         if child_state in reached:
    #             continue
    #         reached.add(child_state)
    #         fringe.push(child_state)
    #         # record parent state, action to go from parent state to now
    #         parent_dict[child_state] = [current_state,parent_to_child_action,parent_to_child_cost]
    #         # if problem.IS-GOAL(s) then return child
    #         if problem.isGoalState(child_state):
    #             path = []
    #             action_lst = []
    #             while True:
    #                 if child_state == problem.getStartState():
    #                     path.append(child_state)
    #                     break
    #                 path.append(child_state)
    #                 action_lst.append( parent_dict[child_state][1])
    #                 child_state = parent_dict[child_state][0]
    #             action_lst = action_lst[::-1]    
    #             path = path[::-1]
    #             return action_lst
    #         # the child_state is not in reached then
    # return []

# dfs sudo code
#     function DEPTH-FIRST-SEARCH(problem):
#           node ← NODE(problem.INITIAL-STATE)
#           if problem.IS-GOAL(node.STATE) then return node
#           frontier ←a LIFO queue (stack) with node as element
#           reached ← {problem.INITIAL-STATE}
#           while not IS-EMPTY(frontier) do
#                   node ← POP(frontier)
#                   for each child in EXPAND(problem, node) do
#                           s ← child.STATE 
#                           if problem.IS-GOAL(s) then return child
#                           if s is not in reached then
#                                   add s to reached
#                                   add child to frontier
#            return failure

    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    fringe = Queue()
    current = (problem.getStartState(), [], [])
    fringe.push(current)
    reached = []
    while not fringe.isEmpty():
        node, path, total = fringe.pop()
        if problem.isGoalState(node):
            return path
        if not node in reached:
            reached.append(node)
            for coord, move, cost in problem.getSuccessors(node):
                fringe.push((coord, path + [move], total + [cost])) 
    return []
# dfs sudo code
#     function BREADTH-FIRST-SEARCH(problem) returns a solution node or failure:
#           node ← NODE(problem.INITIAL-STATE)
#           if problem.IS-GOAL(node.STATE) then return node
#           frontier ←a FIFO queue  with node as element
#           reached ← {problem.INITIAL-STATE}
#           while not IS-EMPTY(frontier) do
#                   node ← POP(frontier)
#                   for each child in EXPAND(problem, node) do
#                           s ← child.STATE 
#                           if problem.IS-GOAL(s) then return child
#                           if s is not in reached then
#                                add s to reached
#                                add child to frontier
#            return failure

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    current = (problem.getStartState(), [], 0)
    fringe.push(current,0)
    reached = []
    while not fringe.isEmpty():
        node, path, cur_cost = fringe.pop()
        if problem.isGoalState(node):
            return path
        if not node in reached:
            reached.append(node)
            for coord, move, cost in problem.getSuccessors(node):
                fringe.update( (coord, path + [move],cur_cost+cost), cur_cost+cost)
    return []


    util.raiseNotDefined()
    #

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    # goal_pos = problem.goal
    from util import PriorityQueue
    fringe = PriorityQueue()
    current = (problem.getStartState(), [], 0)
    fringe.push(current,0)
    reached = []

    while not fringe.isEmpty():
        node, path, cur_cost = fringe.pop()
        if problem.isGoalState(node):
            return path
        if not node in reached:
            reached.append(node)
            for coord, move, cost in problem.getSuccessors(node):
                h_cost = heuristic(coord,problem)
                fringe.update( (coord, path + [move],cur_cost+cost), cur_cost+cost+h_cost)
    return []



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
