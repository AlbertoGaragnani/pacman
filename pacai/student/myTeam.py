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
        self.redTeamEntrances = [(12, 12), (14, 7), (13, 4), (11, 3)]
        self.blueTeamEntrances = [(20, 12), (17, 8), (18, 11), (19, 3)]
        # This variable should act as a workaround to the problem (2) of shadowFunction
        self.iamsafe = False
        self.justGotBack = False

    # --= GHOST FEATURE FUNCTIONS =--#

    # Matching Function
    """
    # Problems:
    # (1) It's possible to bob rush the 'Cross-tile'
    # (2) The ghost tries to go to the closest POI even if the path goes trough the enemy field
    """

    def fShadowFunction(self, gameState, action, enemyIndex):
        # Bug on rush on bottom side from blue team or topside from red team
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

    # Makes a ghost agent that just ate his enemy become a pacman on the closest border location
    def fStartAttack(self, gameState, action, enemyIndex):
        if not gameState.getAgentState(self.index).isPacman() and self.getScore(gameState) <= 0:
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

            if enemyIndex == 0:
                unmatchingIndex = 2
            elif enemyIndex == 1:
                unmatchingIndex = 3
            elif enemyIndex == 2:
                unmatchingIndex = 0
            else:
                unmatchingIndex = 1

            unmatchingEnemyPosition = successor.getAgentState(unmatchingIndex).getPosition()
            distanceToUnmatchingEnemy = self.getMazeDistance(myNextPos, unmatchingEnemyPosition)

            if self.red:
                closestBorderPos = min(redBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if enemyPosition[0] == 30 and not self.justGotBack:  # If the enemy is in the respawn corridor
                    if not currentState.isPacman() and myNextState.isPacman() and distanceToUnmatchingEnemy > 2:
                        return -100
                    else:
                        return self.getMazeDistance(myNextPos, closestBorderPos)
            else:
                closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if enemyPosition[0] == 1 and not self.justGotBack:  # If the enemy is in the respawn corridor
                    if not currentState.isPacman() and myNextState.isPacman() and distanceToUnmatchingEnemy > 2:
                        return -100
                    else:
                        return self.getMazeDistance(myNextPos, closestBorderPos)
            return
        else:
            # We are already a Pacman, so we don't care about this feature
            # Or we are already winning, so we want to camp
            return

    # This function make the ghost attacking an enemy pacman that is one tile away from it
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

    # This function works together with self.iamsafe to solve the problem (2) of shadowFunction
    def fGoSafe(self, gameState, action):
        myPos = gameState.getAgentState(self.index).getPosition()
        successor = self.getSuccessor(gameState, action)
        myNextPos = successor.getAgentState(self.index).getPosition()
        if (self.getScore(gameState) > 0 and not gameState.getAgentState(self.index).isPacman() and not self.iamsafe) or self.justGotBack:
            if self.red:
                if myPos in self.redTeamEntrances:
                    self.iamsafe = True
                    self.justGotBack = False
                distancesToPOI = [self.getMazeDistance(myNextPos, POIPos) for POIPos in self.redTeamEntrances]
                return min(distancesToPOI)
            else:
                if myPos in self.blueTeamEntrances:
                    self.iamsafe = True
                    self.justGotBack = False
                distancesToPOI = [self.getMazeDistance(myNextPos, POIPos) for POIPos in self.blueTeamEntrances]
                return min(distancesToPOI)
        return

    # --= PACMAN FEATURE FUNCTIONS =--#
    def fOffensiveFunction(self, gameState, action, enemyIndex):
        if gameState.getAgentState(self.index).isPacman():
            currentPos = gameState.getAgentState(self.index).getPosition()
            successor = self.getSuccessor(gameState, action)
            myState = successor.getAgentState(self.index)
            myPos = myState.getPosition()
            foodList = self.getFood(successor).asList()
            redBorderPositions = []
            blueBorderPositions = []
            for y in range(1, 15):
                if not gameState.hasWall(15, y) and not gameState.hasWall(16, y):
                    redBorderPositions.append((15, y))
                    blueBorderPositions.append((16, y))

            if self.getScore(successor) > self.getScore(gameState):
                return -100

            if len(foodList):  # and heuristicFunction is True
                closestFood = min(foodList, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if self.red:
                    closestBorderPos = min(redBorderPositions, key=lambda pos: self.getMazeDistance(pos, closestFood))
                    distanceToCaptureSafe = self.getMazeDistance(currentPos, closestFood) + \
                                            self.getMazeDistance(closestFood, closestBorderPos)
                else:
                    closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, closestFood))
                    distanceToCaptureSafe = self.getMazeDistance(currentPos, closestFood) + \
                                            self.getMazeDistance(closestFood, closestBorderPos)
                enemyPosition = gameState.getAgentState(enemyIndex).getPosition()
                distanceToEnemy = self.getMazeDistance(currentPos, enemyPosition)
                if distanceToCaptureSafe < distanceToEnemy + 3:  # This number is quite random
                    minDistanceToFood = min([self.getMazeDistance(myPos, food) for food in foodList])
                    return minDistanceToFood
                else:
                    minDistanceToBorder = self.getMazeDistance(myPos, closestBorderPos)
                    return minDistanceToBorder
            return
        else:
            # We are a ghost, so we don't care about this feature
            return

    # def fEnemyAround(self, gameState, action):
    #     if gameState.getAgentState(self.index).isPacman():
    #

    def fBackToDefense(self, gameState, action):
        if gameState.getAgentState(self.index).isPacman() and self.getScore(gameState) > 0:
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
                if not myNextState.isPacman():
                    self.justGotBack = True
                return self.getMazeDistance(myNextPos, closestBorderPos)
            else:
                closestBorderPos = min(blueBorderPositions, key=lambda pos: self.getMazeDistance(pos, currentPos))
                if not myNextState.isPacman():
                    self.justGotBack = True
                return self.getMazeDistance(myNextPos, closestBorderPos)
        else:
            # Either we are a ghost so we don't care
            # Or we didn't collect the food yet
            return

    # --= CORE FUNCTIONS =--#
    def chooseAction(self, gameState):
        """
        Picks among the actions with the highest return from `ReflexCaptureAgent.evaluate`.
        """

        actions = gameState.getLegalActions(self.index)
        values = [self.evaluate(gameState, a) for a in actions]

        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]

        #New Stuff
        actionToReturn = random.choice(bestActions)
        nextState = self.getSuccessor(gameState, actionToReturn).getAgentState(self.index)
        currentState = gameState.getAgentState(self.index)
        if currentState.isPacman() and not nextState.isPacman():
            self.justGotBack = True

        return actionToReturn

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


class DefensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}

        opponentIndex = self.getOpponents(gameState)[0]
        theOtherOpponent = self.getOpponents(gameState)[1]

        # Shadow function feature
        features['shadow'] = self.fShadowFunction(gameState, action, opponentIndex)

        # Ambush function feature
        features['killPacman'] = self.fAmbushFunction(gameState, action, opponentIndex)

        # Start attack feature function
        features['becomePacman'] = self.fStartAttack(gameState, action, opponentIndex)

        # Go back to defense feature function
        features['becomeGhost'] = self.fBackToDefense(gameState, action)

        # Chase for the closest food feature function
        features['eatFood'] = self.fOffensiveFunction(gameState, action, theOtherOpponent)

        # This feature is a workaround to the problem (2) of shadowFunction
        features['goSafe'] = self.fGoSafe(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'shadow': -5,
            'killPacman': 100,
            'becomePacman': -25,
            'becomeGhost': -10,
            'eatFood': -1,
            'goSafe': -20
        }


class OffensiveAgent(ReflexAgent):

    def __init__(self, index, **kwargs):
        super().__init__(index)

    def getFeatures(self, gameState, action):
        features = {}

        opponentIndex = self.getOpponents(gameState)[1]
        theOtherOpponent = self.getOpponents(gameState)[0]

        # Shadow function feature
        features['shadow'] = self.fShadowFunction(gameState, action, opponentIndex)

        # Ambush function feature
        features['killPacman'] = self.fAmbushFunction(gameState, action, opponentIndex)

        # Start attack feature function
        features['becomePacman'] = self.fStartAttack(gameState, action, opponentIndex)

        # Go back to defense feature function
        features['becomeGhost'] = self.fBackToDefense(gameState, action)

        # Chase for the closest food feature function
        features['eatFood'] = self.fOffensiveFunction(gameState, action, theOtherOpponent)

        # This feature is a workaround to the problem (2) of shadowFunction
        features['goSafe'] = self.fGoSafe(gameState, action)

        return features

    def getWeights(self, gameState, action):
        return {
            'shadow': -5,
            'killPacman': 100,
            'becomePacman': -25,
            'becomeGhost': -10,
            'eatFood': -1,
            'goSafe': -20
        }
