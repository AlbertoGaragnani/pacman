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

    # Matching Function
    def shadowFunction(self, gameState, action):
        # Hard-coding the tiles
        # [0] Top
        # [1] Central
        # [2] Bottom
        tilesToDefend = [(12, 12), (15, 8), (11, 3)]
        #redTeamEntrances = []
        #blueTeamEntrances = []

        #Picking which side we need to defend
        """
        if (self.fGetTeam(gameState, action) == 0):
            tilesToDefend = redTeamEntrances
        else:
            tilesToDefend = blueTeamEntrances
        """
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        opponentIndex = self.getOpponents(gameState)[0]
        enemyPosition = successor.getAgentState(opponentIndex).getPosition()

        enemyToTop = self.getMazeDistance(tilesToDefend[0], enemyPosition)
        enemyToCenter = self.getMazeDistance(tilesToDefend[1], enemyPosition)
        enemyToBottom = self.getMazeDistance(tilesToDefend[2], enemyPosition)

        distances = [enemyToTop, enemyToCenter, enemyToBottom]

        enemyDistanceFromClosestEntrance = min(distances)

        if enemyToTop == enemyDistanceFromClosestEntrance:
            # Defend top
            distanceToTop = self.getMazeDistance(myPos, tilesToDefend[0])
            return distanceToTop
        elif enemyToCenter == enemyDistanceFromClosestEntrance:
            # Defend center
            distanceToCenter = self.getMazeDistance(myPos, tilesToDefend[1])
            return distanceToCenter
        else:
            # Defend bottom
            distanceToBottom = self.getMazeDistance(myPos, tilesToDefend[2])
            return distanceToBottom

    def ambushFunction(self, gameState, action):
        successor = self.getSuccessor(gameState, action)
        opponentIndex = self.getOpponents(gameState)[0]
        enemyPosition = successor.getAgentState(opponentIndex).getPosition()
        
        if enemyPosition == (30, 14):
            return 100

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

    # Gets the team color of our agent. 0 is red, 1 is blue
    def fGetTeam(self, gameState, action):
        if (self.getTeam(gameState) == gameState.getRedTeamIndicies()):
            return 0
        return 1

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
        defenders = [a for a in enemies if not a.isPacman() and a.getPosition() is not None]
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
        stateEval = sum(features[feature] * weights[feature] for feature in features)
        return stateEval


class DefensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}

<<<<<<< HEAD
        # # Computes whether we're on defense (1) or offense (0).
        # features['onDefense'] = self.fRole(gameState, action)
        #
        # # Computes number of invaders we can see.
        # features['numInvaders'] = self.fNumInvaders(gameState, action)
        #
        # # Computes distance of invaders we can see.
        # features['invaderDistance'] = self.fDistInvaders(gameState, action)
        #
        # # Stop Feature
        # features['stop'] = self.fStop(gameState, action)
        #
        # # Reverse Feature
        # features['reverse'] = self.fReverse(gameState, action)
        #
        # # Food Feature
        # features['distanceToFood'] = self.fFoodDist(gameState, action)

        # Shadow function feature
        features['shadow'] = self.shadowFunction(gameState, action)

        # Ambush function feature
        features['killPacman'] = self.ambushFunction(gameState, action)

=======
        successor = self.getSuccessor(gameState, action)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes whether we're on defense (1) or offense (0).
        features['onDefense'] = 1
        if (myState.isPacman()):
            features['onDefense'] = 0

        # Computes distance to invaders we can see.
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman() and a.getPosition() is not None]
        features['numInvaders'] = len(invaders)

        if (len(invaders) > 0):
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            features['invaderDistance'] = min(dists)

        if (action == Directions.STOP):
            features['stop'] = 1

        rev = Directions.REVERSE[gameState.getAgentState(self.index).getDirection()]
        if (action == rev):
            features['reverse'] = 1
        foodList = self.getFood(successor).asList()
        if (len(foodList) > 0):
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance
>>>>>>> origin/Liam
        return features

    def getWeights(self, gameState, action):
        return {
            'distanceToFood': -1,
            'numInvaders': -1000,
            'onDefense': 100,
            'invaderDistance': -100,
            'stop': -100,
            'reverse': -2,
<<<<<<< HEAD
            'distanceToCapsule': -1,
            'shadow': -5,
            'killPacman': 100
=======
            'distanceToCapsule': -1
>>>>>>> origin/Liam
        }


class OffensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}
        successor = self.getSuccessor(gameState, action)
<<<<<<< HEAD
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()

        # # Getting succ score
        # features['successorScore'] = self.getScore(successor)
        #
        # # Computes whether we're on defense (1) or offense (0).
        # # features['onDefense'] = self.fRole(gameState, action)
        #
        # # Computes distance to defenders we can see.
        # features['besideEnemy'] = self.fDistDefenders(gameState, action)
        #
        # # Computes number of invaders
        # features['numInvaders'] = self.fNumInvaders(gameState, action)
        #
        # # Compute distance to the nearest food.
        # features['distanceToFood'] = self.fFoodDist(gameState, action)
        #
        # # Compute distance to the nearest capsule
        # features['distanceToCapsule'] = self.fDistToCapsule(gameState, action)

        opponentIndex = self.getOpponents(gameState)[1]

        # Shadow function feature
        features['shadow'] = self.shadowFunction(gameState, action, opponentIndex)

        # Ambush function feature
        features['killPacman'] = self.ambushFunction(gameState, action, opponentIndex)
=======

        features['successorScore'] = self.getScore(successor)
        myState = successor.getAgentState(self.index)
        myPos = myState.getPosition()
        distToClosestDefender = -1
        # Computes whether we're on defense (1) or offense (0).

        # Computes distance to invaders we can see.
        enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
        invaders = [a for a in enemies if a.isPacman() and a.getPosition() is not None]
        defenders = [a for a in enemies if not a.isPacman() and a.getPosition() is not None]

        distToClosestDefender = \
            min([self.getMazeDistance(myPos, a.getPosition()) for a in defenders])

        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            if (min(dists) < 3) and myState.isPacman() == 0:
                features['besideEnemy'] = 100 / min(dists)

        features['numInvaders'] = len(invaders)

        # Compute distance to the nearest food.
        foodList = self.getFood(successor).asList()

        # This should always be True, but better safe than sorry.
        if (len(foodList) > 0):
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
            features['distanceToFood'] = minDistance

        # Compute distance to the nearest capsule
        capsuleList = self.getCapsules(successor)

        if len(capsuleList) > 0:
            myPos = successor.getAgentState(self.index).getPosition()
            minDistance = min([self.getMazeDistance(myPos, capsule) for capsule in capsuleList])
            if minDistance < 3 and minDistance < distToClosestDefender / 2:
                features['distanceToCapsule'] = 100  # To change
            else:
                features['distanceToCapsule'] = minDistance
>>>>>>> origin/Liam

        return features

    def getWeights(self, gameState, action):
        return {
            'numInvaders': -1000,
            'besideEnemy': 1,
            'successorScore': 100,
            'distanceToFood': -1,
            'stop': -100,
            'reverse': -2,
<<<<<<< HEAD
            'distanceToCapsule': -0.5,
            'shadow': -5,
            'killPacman': 100
        }
=======
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
>>>>>>> origin/Liam
