from pacai.agents.capture.capture import CaptureAgent
from pacai.core.directions import Directions
import random
from pacai.util import util


def createTeam(firstIndex, secondIndex, isRed):
    firstAgent = DefensiveAgent
    secondAgent = OffensiveAgent

    return [
        firstAgent(firstIndex),
        secondAgent(secondIndex),
    ]


class ReflexAgent(CaptureAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index, **kwargs)

    def chooseAction(self, gameState):
        """
        Picks among the actions with the highest return from `ReflexCaptureAgent.evaluate`.
        """

        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        return random.choice(bestActions)

    def getSuccessor(self, gameState, action):
        """
        Finds the next successor which is a grid position (location tuple).
        """

        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()

        if (pos != util.nearestPoint(pos)):
            # Only half a grid position was covered.
            return successor.generateSuccessor(self.index, action)
        else:
            return successor

    def evaluate(self, gameState, action):
        """
        Computes a linear combination of features and feature weights.
        """

        features = self.getFeatures(gameState, action)
        weights = self.getWeights(gameState, action)
        # stateEval = sum(features[feature] * weights[feature] for feature in features)
        stateEval = 0
        for feature in features:
            if features[feature] is not None:
                stateEval += features[feature] * weights[feature]
        return stateEval

    ## --= Feature Functions =-- ##

    # Figures out role of agent
    def fRole(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        if (myState.isPacman()):
            return 0
        return 1

    # Figures out stop action
    def fStop(self, gameState, action):
        if (action == Directions.STOP):
            return 1
        return  # might have to change this function

    # Figures out reverse action
    def fReverse(self, gameState, action):
        rev = Directions.REVERSE[gameState.getAgentState(self.index).getDirection()]
        if (action == rev):
            return 1
        return  # Might have to change this function too

    # Calculates number of invaders nearby
    def fNumInvaders(self, gameState, action):
        successor = self.getSuccessor(gameState, action)

        # Computes distance to invaders we can see.
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman() and a.getPosition() is not None]
        return len(invaders)
        # features['numInvaders'] = len(invaders)

    # Calculates distance of invaders nearby
    def fDistInvaders(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes distance to invaders we can see.
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman() and a.getPosition() is not None]
        numInvaders = self.fNumInvaders(gameState, action)

        if (numInvaders > 0):
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]

            return min(dists)
            # features['invaderDistance'] = min(dists)
        return

    # Calculates distances to foods
    def fFoodDist(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        foodList = self.getFood(successor).asList()

        if (len(foodList) > 0):
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            return minDistance
        return

    def fDistDefenders(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        # invaders = [a for a in enemies if a.isPacman() and a.getPosition() is not None]
        defenders = [a for a in enemies if not a.isPacman() and a.getPosition() is not None]

        # distToClosestDefender = -1
        # distToClosestDefender = min([self.getMazeDistance(myPos, a.getPosition()) for a in defenders])

        tooClose = 3

        if len(defenders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in defenders]
            if (min(dists) < tooClose) and myState.isPacman() == 0:
                return 100 / min(dists)
        return

    def fDistToCapsule(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        capsuleList = self.getCapsules(successor)
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        defenders = [a for a in enemies if not a.isPacman() and a.getPosition() is not None]
        distToClosestDefender = min([self.getMazeDistance(myPos, a.getPosition()) for a in defenders]) \
            if len(defenders) > 0 else -1  # If defenders is empty the value is set to -1

        if len(capsuleList) > 0:

            minCapDistance = min([self.getMazeDistance(myPos, capsule) for capsule in capsuleList])
            if minCapDistance < 3 and minCapDistance < distToClosestDefender / 2:
                return 100  # To change
            else:
                return minCapDistance


class DefensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}

        # Computes whether we're on defense (1) or offense (0).
        features['onDefense'] = self.fRole(gameState, action)

        # Computes number of invaders we can see.
        features['numInvaders'] = self.fNumInvaders(gameState, action)

        # Computes distance of invaders we can see.
        features['invaderDistance'] = self.fDistInvaders(gameState, action)

        # Stop Feature
        features['stop'] = self.fStop(gameState, action)

        # Reverse Feature
        features['reverse'] = self.fReverse(gameState, action)

        # Food Feature
        features['distanceToFood'] = self.fFoodDist(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'distanceToFood': -1,
            'numInvaders': -1000,
            'onDefense': 100,
            'invaderDistance': -100,
            'stop': -100,
            'reverse': -2,
            'distanceToCapsule': -1
        }


class OffensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Getting succ score
        features['successorScore'] = self.getScore(successor)

        # Computes whether we're on defense (1) or offense (0).
        # features['onDefense'] = self.fRole(gameState, action)

        # Computes distance to defenders we can see.
        features['besideEnemy'] = self.fDistDefenders(gameState, action)

        # Computes number of invaders
        features['numInvaders'] = self.fNumInvaders(gameState, action)

        # Compute distance to the nearest food.
        features['distanceToFood'] = self.fFoodDist(gameState, action)

        # Compute distance to the nearest capsule
        features['distanceToCapsule'] = self.fDistToCapsule(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'numInvaders': -1000,
            'besideEnemy': 1,
            'successorScore': 100,
            'distanceToFood': -1,
            'stop': -100,
            'reverse': -2,
            'distanceToCapsule': -0.5
        }

# ASTAR ALGORITHM
# def aStarSearch(problem, heuristic):
#     """
#     Search the node that has the lowest combined cost and heuristic first.
#     """
#     from pacai.util.priorityQueue import PriorityQueue
#
#     # Initialize structs
#     fringe = PriorityQueue()
#     visited = set()
#     pathToNode = []
#
#     fringe.push((problem.startingState(), pathToNode), 0)
#
#     while not fringe.isEmpty():
#         state, path = fringe.pop()
#         if problem.isGoal(state):
#             return path
#
#         if state not in visited:
#             visited.add(state)
#             for succState, action, cost in problem.successorStates(state):
#                 if succState not in visited:
#                     totalCost = problem.actionsCost(path + [action])
#                     fringe.push((succState, path + [action]),
#                                 totalCost + heuristic(succState, problem))
