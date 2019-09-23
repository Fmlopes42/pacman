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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.

    """
    return 10

def myHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.

    """
    #print("myHeuristic")
    #print(problem.isGoalState((1,1)))
    xy2 = problem.goal
    return abs(state[0] - xy2[0]) + abs(state[1] - xy2[1])

def myHeuristic2(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.

    """
    #print("myHeuristic2")
    #print(problem.isGoalState((1,1)))
    xy2 = problem.goal
    return ( (state[0] - xy2[0]) ** 2 + (state[1] - xy2[1]) ** 2 ) ** 0.5

def myHeuristic3(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.

    """
    #canto =[(1,1), (1,2), (2,1), (2,2), (36,36), (35,36), (35,35), (36,35), (1,36),(1,35),(2,36),(2,35),(36,1),(36,2),(35,1),(35,2)]
    canto = []
    for l in range(2):
        for c in range(2):
            canto.append((l,c))
    heru = abs(state[0] - 1) + abs(state[1] - 1)
    #heru=10
    if state in canto:
        #print("sim")
        heru = heru * 0.5
    return heru

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    #frontier = util.PriorityQueue()
    #startState = problem.getStartState()
    #startNode = (startState, ['East'], 0)
    #frontier.push(startNode, 0)

    #currentState, actions, currentCost = frontier.pop()
    #return ['West','West', 'West','West','South','South','East', 'South','South','West','West']

    fronteira = util.PriorityQueue()

    nohExplorado = [] #(state, cost)

    startState = problem.getStartState()
    nohInicial = (startState, [], 0) #(state, action, cost)

    fronteira.push(nohInicial, 0)

    while not fronteira.isEmpty():

        #pega o Noh de menor "custo" na fila
        curEstado, todasAcoes, curCusto = fronteira.pop()

        #Coloca Noh atual na lista de explorados
        nohAtual = (curEstado, curCusto)
        nohExplorado.append((curEstado, curCusto))

        if problem.isGoalState(curEstado):
            #print(todasAcoes)
            return todasAcoes

        else:
            #Lista de Sucessores (successor, action, stepCost) e examina cada um
            sucessores = problem.getSuccessors(curEstado)
            for sucEstado, sucAcao, sucCusto in sucessores:
                novaAcao = todasAcoes + [sucAcao]
                novoCusto = problem.getCostOfActions(novaAcao)
                novoNoh = (sucEstado, novaAcao, novoCusto)

                #Checa se o sucessor jah foi visitado
                jah_foi_explorado = False
                for explorado in nohExplorado:
                    exEstado, exCusto = explorado
                    if (sucEstado == exEstado) and (novoCusto >= exCusto):
                        jah_foi_explorado = True

                #Se nao foi explorado, coloca na fronteira
                if not jah_foi_explorado:
                    fronteira.push(novoNoh, novoCusto + heuristic(sucEstado, problem))
                    nohExplorado.append((sucEstado, novoCusto))

    return todasAcoes



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
