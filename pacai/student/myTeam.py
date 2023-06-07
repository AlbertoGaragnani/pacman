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
        self.RED_RESPAWN_TILES = [(1, 2), (1, 1)]  # The red agents respawn on those
        self.BLUE_RESPAWN_TILES = [(30, 13), (30, 14)]  # The blue agents respawn on those
        """
        # Hard-coding the entrance tiles that are our points of interest (POI)
        # [0] Top
        # [1] Central
        # [2] CenterBottom
        # [3] Bottom
        """
        self.redTeamEntrances = [(12, 12), (14, 8), (13, 4), (11, 3)]
        self.blueTeamEntrances = [(20, 12), (17, 8), (18, 11), (19, 3)]

    # Matching Function
    def fShadowFunction(self, gameState, action, enemyIndex):
        if not gameState.getAgentState(self.index).isPacman():
            tilesToDefend = []

            # Picking which side we need to defend
            if self.red:
                tilesToDefend = self.redTeamEntrances
            else:
                tilesToDefend = self.blueTeamEntrances

            successor = self.getSuccessor(gameState, action)
            myState = successor.getAgentState(self.index)
            myPos = myState.getPosition()
            enemyPosition = successor.getAgentState(enemyIndex).getPosition()

            enemyToTop = self.getMazeDistance(tilesToDefend[0], enemyPosition)
            enemyToCenter = self.getMazeDistance(tilesToDefend[1], enemyPosition)
            enemyToCenterBottom = self.getMazeDistance(tilesToDefend[2], enemyPosition)
            enemyToBottom = self.getMazeDistance(tilesToDefend[3], enemyPosition)

            distances = [enemyToTop, enemyToCenter, enemyToCenterBottom, enemyToBottom]

            enemyDistanceFromClosestEntrance = min(distances)

            if enemyToTop == enemyDistanceFromClosestEntrance:
                # Defend top
                distanceToTop = self.getMazeDistance(myPos, tilesToDefend[0])
                return distanceToTop
            elif enemyToCenter == enemyDistanceFromClosestEntrance:
                # Defend center
                distanceToCenter = self.getMazeDistance(myPos, tilesToDefend[1])
                return distanceToCenter
            elif enemyToCenterBottom == enemyDistanceFromClosestEntrance:
                # Defend center bottom
                distanceToCenterBottom = self.getMazeDistance(myPos, tilesToDefend[2])
                return distanceToCenterBottom
            else:
                # Defend bottom
                distanceToBottom = self.getMazeDistance(myPos, tilesToDefend[3])
                return distanceToBottom
        else:
            # We are a Pacman, so we don't care about this feature
            return

    # This causes problems
    def fBeatDummyAgent(self, gameState, action):
        if self.red:
            if gameState.getScore() == 0 and action == Directions.SOUTH and \
                    gameState.getAgentState(self.index).getPosition() == self.redTeamEntrances[1]:  # Center
                return 1
            else:
                return 0
        else:
            if gameState.getScore() == 0 and action == Directions.SOUTH and \
                    gameState.getAgentState(self.index).getPosition() == self.blueTeamEntrances[1]:  # Center
                return 1
            else:
                return 0

    # Makes a ghost agent that just ate his enemy become a pacman on the closest border location
    def fStartAttack(self, gameState, action, enemyIndex):
        if not gameState.getAgentState(self.index).isPacman() and gameState.getScore() <= 0:
            successor = self.getSuccessor(gameState, action)
            currentState = gameState.getAgentState(self.index)
            currentPos = currentState.getPosition()
            myNextState = successor.getAgentState(self.index)
            myNextPos = myNextState.getPosition()
            enemyPosition = successor.getAgentState(enemyIndex).getPosition()
            redBorderPositions = []
            blueBorderPositions = []
            for y in range(1, 15):
                if not gameState.hasWall(15, y) and not gameState.hasWall(16, y):
                    redBorderPositions.append((15, y))
                    blueBorderPositions.append((16, y))

            if self.red:
                closestBorderPos = min(redBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if enemyPosition[0] == 30:  # If the enemy is in the respawn corridor
                    if not currentState.isPacman() and myNextState.isPacman():
                        return -100
                    else:
                        return self.getMazeDistance(myNextPos, closestBorderPos)
            else:
                closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if enemyPosition[0] == 1:  # If the enemy is in the respawn corridor
                    if not currentState.isPacman() and myNextState.isPacman():
                        return -100
                    else:
                        return self.getMazeDistance(myNextPos, closestBorderPos)
            return 0
        else:
            # We are already a Pacman, so we don't care about this feature
            # Or we are already winning, so we want to camp
            return

    def fAmbushFunction(self, gameState, action, enemyIndex):
        if not gameState.getAgentState(self.index).isPacman():
            successor = self.getSuccessor(gameState, action)
            enemyPosition = successor.getAgentState(enemyIndex).getPosition()

            if self.red:
                if enemyPosition == self.BLUE_RESPAWN_TILES[0] or enemyPosition == self.BLUE_RESPAWN_TILES[1]:
                    return 100
            else:
                if enemyPosition == self.RED_RESPAWN_TILES[0] or enemyPosition == self.RED_RESPAWN_TILES[1]:
                    return 100
        else:
            # We are a Pacman, so we don't care about this feature
            return

    def fOffensiveFunction(self, gameState, action):
        if gameState.getAgentState(self.index).isPacman():
            successor = self.getSuccessor(gameState, action)
            myState = successor.getAgentState(self.index)
            myPos = myState.getPosition()
            foodList = self.getFood(successor).asList()

            if successor.getScore() > gameState.getScore():
                return -100
            safety = self.offensiveHeuristic(successor) # FOR TESTING
            if len(foodList):  # and heuristicFunction is True
                minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
                return minDistance
            return
        else:
            # We are a ghost, so we don't care about this feature
            return

    def fBackToDefense(self, gameState, action):
        if gameState.getAgentState(self.index).isPacman() and gameState.getScore() > 0:
            successor = self.getSuccessor(gameState, action)
            currentState = gameState.getAgentState(self.index)
            currentPos = currentState.getPosition()
            myNextState = successor.getAgentState(self.index)
            myNextPos = myNextState.getPosition()
            redBorderPositions = []
            blueBorderPositions = []
            for y in range(1, 15):
                if not gameState.hasWall(15, y) and not gameState.hasWall(16, y):
                    redBorderPositions.append((15, y))
                    blueBorderPositions.append((16, y))

            if self.red:
                closestBorderPos = min(redBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                return self.getMazeDistance(myNextPos, closestBorderPos)
            else:
                closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                return self.getMazeDistance(myNextPos, closestBorderPos)
        else:
            # Either we are a ghost so we don't care
            # Or we didn't collect the food yet
            return

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

    # NEW
    def fBackToDefensePos(self, gameState, action):
        
        successor = self.getSuccessor(gameState, action)
        currentState = gameState.getAgentState(self.index)
        currentPos = currentState.getPosition()
        myNextState = successor.getAgentState(self.index)
        myNextPos = myNextState.getPosition()
        redBorderPositions = []
        blueBorderPositions = []
        for y in range(1, 15):
            if not gameState.hasWall(15, y) and not gameState.hasWall(16, y):
                redBorderPositions.append((15, y))
                blueBorderPositions.append((16, y))

        if self.red:
            closestBorderPos = min(redBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
            return self.getMazeDistance(myNextPos, closestBorderPos), closestBorderPos
        else:
            closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
            return self.getMazeDistance(myNextPos, closestBorderPos), closestBorderPos
        
    # NEW
    def offensiveHeuristic(self, gameState):
        # Find nearest capsule
        # If we can reach that capsule before ghost reaches us AND we can return home without ghost in our path, then go get that capsule
        # Else go back to closest "special" zone
        # successor = self.getSuccessor(gameState, Directions.STOP)

        myState = gameState
        myPos = myState.getAgentPosition(self.index)

        foodList = self.getFood(myState).asList()
        closestFoodPos = (0, 0)
        closestFoodDistance = float("inf")
        for food in foodList:
            dist = self.getMazeDistance(myPos, food)
            if dist < closestFoodDistance:
                closestFoodDistance = dist
                closestFoodPos = food
        print("myState")
        print(myState)
        print("problem")
        problem = self.problemCreator(myState)
        print(problem)
        print("food")
        print(food)
        pathToFood, destinationPos = self.aStarSearch(closestFoodPos, myPos, problem)
        print("after first aStarSearch()")
        distanceToBorder, borderPos = self.fBackToDefensePos(gameState, Directions.STOP)
        pathHome, myState = self.aStarSearch(borderPos, destinationPos, problem)
        
        print("after aStarSearch()")
        print("pathToFood")
        print(pathToFood)
        print("pathHome")
        print(pathHome)

        enemies = [myState.getAgentState(i) for i in self.getOpponents(myState)]
        defenders = [a for a in enemies if not a.isPacman() and a.getPosition() is not None]
        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in defenders]
        enemyPaths = [self.aStarSearch(myPos, a.getPosition()) for a in defenders]
        print("end of heuristic")

        return True
    # NEW
    def problemCreator(self, gameState):
        from pacai.util.queue import Queue

        # Initialize structs
        fringe = Queue()
        visited = set()
        problem = dict()
        pathToNode = []

        fringe.push((gameState, pathToNode))

        while not fringe.isEmpty():
            state, path = fringe.pop()
            if state.getAgentPosition(self.index) not in visited:
                visited.add(state.getAgentPosition(self.index))
                actions = state.getLegalActions()
                for action in actions:
                    successor = self.getSuccessor(state, action)
                    try:
                        problem[state.getAgentPosition(self.index)] += (successor.getAgentPosition(self.index), action)
                    except:
                        problem[state.getAgentPosition(self.index)] = (successor.getAgentPosition(self.index), action)
                    if successor.getAgentPosition(self.index) not in visited:
                        fringe.push((successor, path + [action]))
        return problem
    # NEW               
    # ASTAR ALGORITHM
    def aStarSearch(self, goalPos, pos, problem):
        """
        Search the node that has the lowest combined cost and heuristic first.
        """
        from pacai.util.priorityQueue import PriorityQueue
        import math
        # Initialize structs
        fringe = PriorityQueue()
        visited = set()
        pathToNode = []

        fringe.push((pos, pathToNode), 0)

        while not fringe.isEmpty():
            statePos, path = fringe.pop()
            if statePos == goalPos:
                return path, statePos
            if statePos not in visited:
                visited.add(statePos)
                succState = (0, 0)
                for successor in problem[statePos]:
                    if isinstance(successor[0], int):
                        succState = successor
                        continue
                    else:
                        action = successor
                    if succState not in visited:
                        totalCost = len(path + [action])
                        fringe.push((succState, path + [action]), totalCost)


class DefensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}

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

        opponentIndex = self.getOpponents(gameState)[0]

        # Shadow function feature
        features['shadow'] = self.fShadowFunction(gameState, action, opponentIndex)

        # Ambush function feature
        features['killPacman'] = self.fAmbushFunction(gameState, action, opponentIndex)

        # Beat dummy agent feature
        # features['beatDummyAgent'] = self.fBeatDummyAgent(gameState, action)

        # Start attack feature function
        features['becomePacman'] = self.fStartAttack(gameState, action, opponentIndex)

        # Go back to defense feature function
        features['becomeGhost'] = self.fBackToDefense(gameState, action)

        # Chase for the closest food feature function
        features['eatFood'] = self.fOffensiveFunction(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'distanceToFood': -1,
            'numInvaders': -1000,
            'onDefense': 100,
            'invaderDistance': -100,
            'stop': -100,
            'reverse': -2,
            'distanceToCapsule': -1,
            'shadow': -5,
            'killPacman': 100,
            'beatDummyAgent': 50,
            'becomePacman': -25,
            'becomeGhost': -10,
            'eatFood': -1
        }


class OffensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}
        successor = self.getSuccessor(gameState, action)
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
        features['shadow'] = self.fShadowFunction(gameState, action, opponentIndex)

        # Ambush function feature
        features['killPacman'] = self.fAmbushFunction(gameState, action, opponentIndex)

        # Beat dummy agent feature
        # features['beatDummyAgent'] = self.fBeatDummyAgent(gameState, action)

        # Start attack feature function
        features['becomePacman'] = self.fStartAttack(gameState, action, opponentIndex)

        # Go back to defense feature function
        features['becomeGhost'] = self.fBackToDefense(gameState, action)

        # Chase for the closest food feature function
        features['eatFood'] = self.fOffensiveFunction(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'numInvaders': -1000,
            'besideEnemy': 1,
            'successorScore': 100,
            'distanceToFood': -1,
            'stop': -100,
            'reverse': -2,
            'distanceToCapsule': -0.5,
            'shadow': -5,
            'killPacman': 100,
            'beatDummyAgent': 50,
            'becomePacman': -25,
            'becomeGhost': -10,
            'eatFood': -1
        }