##################################################################################
# Copyright (c) 2010, 2011, 2012, 2013, Daniel Urieli, Peter Stone
# University of Texas at Austin
# All right reserved
# 
# Based On:
# 
# Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the University of Amsterdam nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##################################################################################



from vec2d import *
from lineline import *
from math import *

import geometry
from actions import *
import state
import rrtTree
import random

#TODO: merge into AgentPIDTactic? no good reason to separate them
class AgentPIDTacticImpl:
  """
  Implements a tactic in which an agent uses a PID controler
  to reach its goal position.
  """

  def __init__(self, agentIndex, goalPos, rulesOfTheSea):

    self.shipIndex = agentIndex
    self.goalPos = goalPos
    self.rulesOfTheSea = rulesOfTheSea
    # TODO: send these constants as params
    self.anglePID = 0.5 #1 #0.2
    self.distPID = 0.1# 0.2
    # Added integral term to the controller because of strong winds
    self.distIConst = 0.00001#0.01
    self.distIntergral = 0


  def composeAction(self, agentWorldModel):
    """
    Implements the abstract method.
    """
    # Rules of the sea - check for potential colisions with other ships
    if self.rulesOfTheSea:
      shipsToAvoid = self.getShipsOnACollisionPath(agentWorldModel)
      if len(shipsToAvoid) > 0:
        return self.composeAvoidingAction(shipsToAvoid, agentWorldModel)
      else:
        # If no collision danger, compose action as usual
        return self.composePIDAction(agentWorldModel)
    else: 
      # If no collision danger, compose action as usual
      return self.composePIDAction(agentWorldModel)

  def composePIDAction(self, agentWorldModel):
    """
    Composes a PID-controller-based action in order to reach the goal.
    This function is by default used in unless there is a collision danger.
    """
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    estimatedPos = (estimatedState.x, estimatedState.y)
   
    distanceToGoal = geometry.distance2D(estimatedPos, self.goalPos)
    angleFixToGoal = self.getAngleFixToGoal(estimatedState, self.goalPos)


    # If at goal - stop
    if distanceToGoal <= 10:
      action = CompositeAction()
      action.appendAction(PrimitiveAction("stop", []))
      return action

    # accumulate data for the integral part of the PI controller
    self.distIntergral += distanceToGoal

    action = CompositeAction()
    # Steering - (Proportinal controller)
    if abs(angleFixToGoal) > 10: 
      requiredSteering = angleFixToGoal * self.anglePID
      # TODO: agent should have a pointer to the ship so he could 
      # know the max speed rather than hand cropping it here to between [-90, 90]
      steering = max(min(requiredSteering, 90), -90)
      action.appendAction(PrimitiveAction("setSteering", [steering]))
      

    # Speed - PI (no D) controller
    requiredSpeed = distanceToGoal * self.distPID + self.distIConst * self.distIntergral
    # TODO: agent should have a pointer to the ship so he could know the max speed and so on
    speed = max(min(requiredSpeed, 40), -10)


    action.appendAction(PrimitiveAction("setEngineSpeed", [speed]))  

    return action


  def getAngleFixToGoal(self, estimatedState, goal):
    """
    According to the estimation, how much should we turn
    to point at the goal

    Keyword arguments:
    estimatedState -- estimated ShipExternalState

    returns:
    The angle we should turn
    """
    estimatedPos = (estimatedState.x, estimatedState.y)
    desiredOrientation = geometry.orientation2D(estimatedPos, goal)
    estimatedOrientation = estimatedState.orientation
    return geometry.computeShortestTurn(estimatedOrientation, desiredOrientation)

  def getShipsOnACollisionPath(self, agentWorldModel):
    """
    Compute a list of ships that are on path to potential collision 
    with the agent's ship.

    @type  agentWorldModel: AgentWorldModel 
    @param agentWorldModel: The belief state of the agent

    @rtype: a list of ShipExternalState
    @return: External states (position, orientation...) of ships that are on a path to potential collision with the agent's ship.
    """
    # extract my position, orientation, speed
#    print 'shipIndex', self.shipIndex
    estimatedSelfState = agentWorldModel.getEstimatedExtShipState()
    myPos = Vec2d(estimatedSelfState.x, estimatedSelfState.y)
#    print 'myPos', myPos
    myOrient = estimatedSelfState.orientation
#    print 'myOrient', myOrient
    mySpeed = estimatedSelfState.speed
#    print 'mySpeed', mySpeed
    
    # create my line of motion
    myDirection = Vec2d(cos(radians(myOrient)), sin(radians(myOrient))) 
#    print 'myDirection', myDirection
    myLineP1 = myPos - myDirection * 10000
    myLineP2 = myPos + myDirection * 10000


    # check potential collisions with any other ship
    dangerShipsStates = []
    externalShipStates = agentWorldModel.getEstimatedStatesOfOtherShips(self.shipIndex)
    for state in externalShipStates:
      # extract other agent's position, orientation, speed
      otherPos = Vec2d(state.x, state.y)
#      print 'otherPos', otherPos
      otherOrient = state.orientation
#      print 'otherOrient', otherOrient
      otherSpeed = state.speed
#      print 'otherSpeed', otherSpeed

      # create the other ship's line of motion
      otherDirection = Vec2d(cos(radians(otherOrient)), sin(radians(otherOrient))) 
#      print 'otherDirection', otherDirection
      otherLineP1 = otherPos - otherDirection * 10000
      otherLineP2 = otherPos + otherDirection * 10000


  
      # check if coming towards each other 
      globalAngleToOtherShip = (otherPos - myPos).get_angle()
      relativeAngleToOtherShip = geometry.computeShortestTurn(myOrient, globalAngleToOtherShip)
#      print 'relativeAngleToOtherShip', relativeAngleToOtherShip
      if 45 > relativeAngleToOtherShip > -45:
        relativeOrient = geometry.computeShortestTurn(myOrient, otherOrient)
#        print 'relativeOrient', relativeOrient
        if abs(relativeOrient) > 135:
          distToOther = (otherPos - myPos).get_length()
#          print 'distToOther', distToOther
          sumSpeeds = mySpeed + otherSpeed
#          print 'sumSpeeds', sumSpeeds
          timeToCollision = distToOther / sumSpeeds if sumSpeeds > 0 else 999999
#          print 'timeToCollision', timeToCollision
          # TODO: tune threshold and merge with threshold from below
          if timeToCollision < 60:
            dangerShipsStates.append(state)
            continue
            


      # check if lines intersects in any way
      intersection = lineline(myLineP1, myLineP2, otherLineP1, otherLineP2)
#      print 'intersection', intersection
      # if intersection is None then there's no intersection => nothing to do 
      if intersection:
        myDistToIntersect= (intersection - myPos).get_length()
#        print 'myDistToIntersect', myDistToIntersect
        otherDistToIntersect = (intersection - otherPos).get_length()
#        print 'otherDistToIntersect', otherDistToIntersect
        # if intersection is ahead of us or not too far behind us
        # TODO: parametrize tune threshold of -50 
        if myDistToIntersect > -50 and otherDistToIntersect > -50:
          myTimeToIntersect = myDistToIntersect / mySpeed if mySpeed > 0 else 999999
#          print 'myTimeToIntersect', myTimeToIntersect
          otherTimeToIntersect = otherDistToIntersect / otherSpeed if otherSpeed > 0 else 999999
#          print 'otherTimeToIntersect', otherTimeToIntersect

          # TODO: tune thresholds
          if myTimeToIntersect < 60 and otherTimeToIntersect < 60 and abs(myTimeToIntersect - otherTimeToIntersect) < 30:
            # collision danger
            dangerShipsStates.append(state)

#    print 'dangerShipsStates', dangerShipsStates
    return dangerShipsStates 
      
      


  def composeAvoidingAction(self, shipsToAvoid, agentWorldModel):
    """
    Compute an action to take when there is a collision danger.

    @type  shipsToAvoid: a list of ShipExternalState
    @param shipsToAvoid: External states (position, orientation...) of ships that are on a path to potential collision with the agent's ship.

    @type  agentWorldModel: AgentWorldModel 
    @param agentWorldModel: The belief state of the agent

    @rtype: CompositeAction
    @return: Action to take in order to avoid collision
    """
    # In general, if no ships on the right, turn right. Otherwise stop.
    estimatedSelfState = agentWorldModel.getEstimatedExtShipState()
    myPos = Vec2d(estimatedSelfState.x, estimatedSelfState.y)
    myOrient = estimatedSelfState.orientation

    turnRight = False
    stop = False
    for state in shipsToAvoid:
      otherShipPos = Vec2d(state.x, state.y)
      otherShipOrientation = state.orientation
      globalAngleToOtherShip = (otherShipPos - myPos).get_angle()
      relativeAngleToOtherShip = geometry.computeShortestTurn(myOrient, globalAngleToOtherShip)
      relativeOrient = geometry.computeShortestTurn(myOrient, otherShipOrientation)
      if abs(relativeAngleToOtherShip) < 45:
        if abs(relativeOrient) > 135:
          # facing each other, turn right
          turnRight = True
        elif abs(relativeOrient) < 45:
          # behind the other, stop
          stop = True
      elif 45 < relativeAngleToOtherShip < 135:
#      elif -135 < relativeAngleToOtherShip < -45: # TODO: as the 0,0 is top left (upside down screen) "left" and "right" are opposite, so if find solution to this problem, restore this line
        # on my right, stop
        stop = True
      elif -135 < relativeAngleToOtherShip < -45:
#      elif 45 < relativeAngleToOtherShip < 135: # TODO: as the 0,0 is top left (upside down screen) "left" and "right" are opposite, so if find solution to this problem, restore this line
        turnRight = True
      else:
        # behind me, do nothing
        pass


    if stop:
      # on my right, stop
      action = CompositeAction()
#      print 'STOPPING'
      # TODO: instead of a fixed steering, we should have some kind 
      # of PID based on the distance to collision point
      action.appendAction(PrimitiveAction("setEngineSpeed", [-10]))
      return action

    if turnRight:
      # on my front or left, turn right
      action = CompositeAction()
      # TODO: instead of a fixed steering, we should have some kind 
      # of PID based on the distance to collision point
#      print 'TURNING TO THE RIGHT'
      action.appendAction(PrimitiveAction("setSteering", [30]))
      return action

    # if reached here, all ships are behind us, nothing special todo
    return self.composePIDAction(agentWorldModel)


#TODO: merge into AgentRRTTactic? no good reason to separate them
class AgentRRTTacticImpl:
  """
  This class implements an RRT based partolling agent tactic.
  Given a start position and a goal position, it runs a simplistic RRT 
  algorithm, and extracts the resulting path.
  """
  def __init__(self, agentIndex, startPos, goalPos, obstacleList, rulesOfTheSea):
    """
    @type  agentIndex: int
    @param agentIndex: the index of the patroling agent (like an agent-id)

    @type  startPos: 2-tuple
    @param startPos: start position for the RRT computation

    @type  goalPos: 2-tuple
    @param goalPos: goal position for the RRT computation

    @type  rulesOfTheSea: boolean
    @param rulesOfTheSea: Tells whether to respect the rules of the sea
    """

    self.agentIndex = agentIndex
    self.startPos = state.ShipExternalState(startPos[0], startPos[1], 0, 0, 0)
    self.goalPos = state.ShipExternalState(goalPos[0], goalPos[1], 0, 0, 0)
    self.obstacleList = obstacleList
    self.rulesOfTheSea = rulesOfTheSea
    self.actionIndex = 0




    # TODO: Some hard-coded constants that should probably be sent as params

    # Threshold to determine whether at goal
    self.angleThreshold = 10
    self.distanceThreshold = 10 

    # borders for sampling
    xStart, yStart = startPos
    xEnd, yEnd = goalPos
    # randint expects an int
    self.minX = int(round(min(xStart, xEnd) - 200))
    self.maxX = int(round(max(xStart, xEnd) + 200))
    self.minY = int(round(min(yStart, yEnd) - 200))
    self.maxY = int(round(max(yStart, yEnd) + 200))
    self.minAng = 0
    self.maxAng = 360

    # step-fraction towards a new sampled state
    self.epsilon = 0.1 


    
    # Compute the RRT path 
    # init a tree that is filled inside self.rrtPath()  
    #(TODO: change distanceUnit and angleUnit?)
    self.tree = rrtTree.RRTTreeXYOrientationEuclidDist(self.startPos, distanceUnit=10, angleUnit=5)
    self.path = self.rrtPath(self.startPos, self.goalPos)


  def getPath(self):
    """
    Returns the path of points along which the agent patrols.

    @rtype: list of 2-tuples
    @return: a list of (x, y) points representing the patrol path
    """
    return self.path

  def rrtPath(self, startPos, goalPos):
    """
    Computes an RRT path from startPos to goalPos.

    @type  startPos: 2-tuple
    @param startPos: start position for the RRT computation

    @type  goalPos: 2-tuple
    @param goalPos: goal position for the RRT computation

    @rtype: list of 2-tuples
    @return: a list of (x, y) points representing the patrol path
    """
    # create initial node
    initialExternalState = startPos


    # search
    currentState = initialExternalState
    while not self.isGoalState(currentState):
      currentState = self.extend(self.tree, self.chooseRandomState())

    nodesPath = self.tree.getNodesPathTo(currentState)
    path = [(n.state.x, n.state.y) for n in nodesPath]
    return path
    

  def extend(self, tree, randomState):
    """
    The EXTEND function from the paper.
    Here, in this simple version of the algorithms, we just 
    take an epsilon step towards the randomState, without checking
    the all intermediate states are collision-free.
    We also assume holonomic problem in which the computation
    of the edge (the action that takes us to the new state) 
    is just a simple vector calculation.

    
    @type  randomState: ShipExternalState
    @param randomState: A state that was randomly sampled in space, towards which we take an epsilon step.

    @rtype: ShipExternalState
    @return: The new state that was added to the tree.
    """
    closestNode = tree.findClosestNode(randomState)
    closestState = closestNode.state

    # if intersects some obstacle - reject point
    for obst in self.obstacleList:
      A = Vec2d(closestState.x, closestState.y)
      B = Vec2d(randomState.x, randomState.y)
      C, D = obst.getBorder()
      C = Vec2d(C)
      D = Vec2d(D)

      if lineline(A, B, C, D):
        #intersects!
        return None


    # Take an epsilon step towards randomState
    # TODO: need to normalize attributes with variance
    closestStateX = closestState.x
    closestStateY = closestState.y
    closestStateOrient = closestState.orientation
    newX = (randomState.x - closestStateX) * self.epsilon + closestStateX
    newY = (randomState.y - closestStateY) * self.epsilon + closestStateY
    newOrient = (randomState.orientation - closestStateOrient) * self.epsilon + closestStateOrient

    # Compute the edge - the action to take in order to get to this state
    # TODO: check that the path to this state is legal
    xDiff = newX - closestStateX
    yDiff = newY - closestStateY

    if closestNode.parent != None:
      oldOrientation = degrees( atan2(closestStateY - closestNode.parent.state.y,
                                closestStateX - closestNode.parent.state.x) )
    else:
      oldOrientation = closestStateOrient
    # Turn from the old direction to the new direction
    steeringToApply = degrees( atan2(yDiff, xDiff) ) - oldOrientation 

    speedToApply = sqrt( xDiff * xDiff + yDiff * yDiff )
    action = CompositeAction()
    action.appendAction(PrimitiveAction("setEngineSpeed", [speedToApply]))
    action.appendAction(PrimitiveAction("setSteering", [steeringToApply]))

    # Add the node to the tree
    newState = state.ShipExternalState(newX, newY, newOrient, speedToApply, 0) 
    parent = closestNode
    edge = action
    newNode = rrtTree.RRTNode(newState, parent, edge)
    tree.addNode(newNode)

    return newState
    
  def isGoalState(self,shipState):
    "Is the ship within some threshold from the goal."

    if not shipState:
      return False

    point1 = (shipState.x, shipState.y)
    point2 = (self.goalPos.x, self.goalPos.y)
    distance = geometry.distance2D(point1, point2)
    if distance > self.distanceThreshold:
      return False

#    # ignore angle for now
#    angle1 = shipState.orientation
#    angle2 = self.goalPos.orientation
#    if abs(computeShortestTurn(angle2,angle1)) > self.angleThreshold:
#      return False

    return True


  def chooseRandomState(self):
    "Samples a random state with a bias towards the goal."
    if random.random() < 0.05:
      return self.goalPos
    else:
      return state.ShipExternalState(random.randint(self.minX, self.maxX), 
                                     random.randint(self.minY, self.maxY),
                                     random.randint(self.minAng, self.maxAng),
                                     speed = -1, angularSpeed = -1) # speed should be ignored




##########################################################################
# An abstract class for low-level navigation technique. 
# While the AgentStrategy class encapsulates the agent's
# complete strategy, the NavigateToPointTactic class
# only cares about how to reach the next goal point.
##########################################################################
      

class NavigateToPointTactic:
  """
  This is a base class for low level code that navigates to specific point.
  """
  def composeAction(self, agentWorldModel):
    "Given the (inferred) state of the world, decide on a set of actions."
    abstract

  def done(self, agentWorldModel):
    "Is goal reached."
    abstract

  def atPoint(self, estimatedPos, point):
    distanceToGoal = geometry.distance2D(estimatedPos, point)
    #if distanceToGoal <= 20:
    return  (distanceToGoal <= 50)

  def getGoalPos(self):
    """
    Navigation assumed to have a goal position. 
    Thie method returns this goal.
    """
    abstract


#class AgentNullTactic(NavigateToPointTactic):
#  """
#  Do nothing, always return done() == True. Used for initialization.
#  """
#  def composeAction(self, agentWorldModel):
#    "Implements abstract method."
#    # should have no action implemented
#    abstract
#
#  def done(self, agentWorldModel):
#    "Implements abstract method."
#    return True


class AgentPIDTactic(NavigateToPointTactic):
  """
  Low level tactic for PID based navigation to a point
  """
  def __init__(self, agentIndex, goalPoint, rulesOfTheSea):
    # TODO: eliminate self.worker and merge AgentPIDTacticImpl
    #       to here?
    self.goalPoint = goalPoint
    self.worker = AgentPIDTacticImpl(agentIndex, goalPoint, rulesOfTheSea)

  def composeAction(self, agentWorldModel):
    "Implements abstract method."
    return self.worker.composeAction(agentWorldModel)

  def done(self, agentWorldModel):
    "Implements abstract method."
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    estimatedPos = (estimatedState.x, estimatedState.y)
    return self.atPoint(estimatedPos, self.goalPoint)

  def getGoalPos(self):
    "Implements abstract method."
    return self.goalPoint

class AgentRRTTactic(NavigateToPointTactic):
  """
  Low level tactic for RRT based navigation to a point
  """
  def __init__(self, agentIndex, 
                     estimatedPos, 
                     goalPoint, 
                     estimatedObstaclesList,
                     rulesOfTheSea):

    self.agentIndex = agentIndex
    self.rrtBuilder = AgentRRTTacticImpl(agentIndex, 
                                        estimatedPos, 
                                        goalPoint, 
                                        estimatedObstaclesList,
                                        rulesOfTheSea)
    self.rrtPath = self.rrtBuilder.path
    # the two atPoint() tests should be consistent
    # and therefore we set the goal point to
    # the last point in the rrt rather than the real goal
    self.goalPoint = self.rrtPath[-1]# goalPoint
    self.nextRRTPointIndex = 0 
    nextPoint = self.rrtPath[self.nextRRTPointIndex]
    self.pidTactic = AgentPIDTactic(agentIndex, nextPoint, rulesOfTheSea)
    self.rulesOfTheSea = rulesOfTheSea

  def composeAction(self, agentWorldModel):
    "Implements abstract method."
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    estimatedPos = (estimatedState.x, estimatedState.y)
    nextPoint = self.rrtPath[self.nextRRTPointIndex]
    if self.atPoint(estimatedPos, nextPoint):  
      # set next point as target
      self.nextRRTPointIndex += 1
      nextPoint = self.rrtPath[self.nextRRTPointIndex]
      self.pidTactic = AgentPIDTactic(self.agentIndex, 
                                      nextPoint, 
                                      self.rulesOfTheSea)
    return self.pidTactic.composeAction(agentWorldModel)

  def done(self, agentWorldModel):
    "Implements abstract method."
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    estimatedPos = (estimatedState.x, estimatedState.y)
    return self.atPoint(estimatedPos, self.goalPoint)

  def getGoalPos(self):
    "Implements abstract method."
    return self.goalPoint
