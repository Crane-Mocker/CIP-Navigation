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



import copy
import random
import os
import sys
import operator
from math import *
import numpy

from vec2d import *
from lineline import *

import geometry
import rrtTree
import perceptModels
from actions import *
import state
from messages import PrimitiveMessage, CompositeMessage
import obstacleAvoidance
from navigationTactics import *
from scripts import heuristicDivide

########################################################################
#
# This file defines different types of agents.
# An agent is defined by:
# - The way it models the world
# - The way it process percepts for this given world model
# - The set of legal actions available to it
# - It's strategy for choosing actions, based on the set of legal actions,
#   and based on its world model.
#
# The AgentWorldModel decouples the perception part from the action 
# part, thus allowing us to mix any combination of percepts, 
# percept-processing-algorithm, world-model, legal-action-set,
# and decision-making-strategy.
#
# Implementation-wise, an agent has two components:
# - AgentWorldModel: implements a specific worldmodel, and percept processing
# - AgentStrategy: for a given AgentWorldModel and legal actions,
#                  defines a strategy for choosing actions
#
########################################################################




########################################################################
# AgentWorldModel class. Implements different models an agent 
# can have about the world.
########################################################################
class AgentWorldModel:
  """
  This class is a base class for all the models an agent
  can have about the world.
  A subclass should implement a specific model of the world,
  and a specific way to process percepts for this model.
  There can be multiple ways to process percepts for any
  given model, so an example class hierarchy could be:
    
                  -------------------
                  | AgentWorldModel |
                  -------------------
                      /            \
                ----------          ----------
                | Model1 |          | Model2 |
                ----------          ----------
                /       \                 \ 
  -------------------  ------------------  .
  |with method 1 to |  |with method 2 to|   .
  |processPercepts  |  |processPercepts |    .
  |for Model1       |  |for Model1      |
  -------------------  ------------------
  """
  def __init__(self):
    """
    Every world model contains a queue for incoming messages. 
    Note that outgoing msgs are inside strategy.
    TODO: does this design need change? 
    """
    self.incomingMsgs = CompositeMessage()

  # Abstract methods
  def processPercepts(self, perceptsList):
    """
    For a given concrete model, this method
    implements a specific method of updating this 
    model from percepts.
    This is just an extra layer of flexibility, in case
    there are multiple methods to update percepts for the 
    same model.

    @type  perceptsList: PerceptList
    @param perceptsList: A list of percepts to be processed
    """
    abstract

  def getEstimatedStatesOfOtherShips(self, selfIndex):
    """
    Get a list of estimated ShipExternalState of all other ships
    in the system.

    @type  selfIndex: int
    @param selfIndex: The index of the ship controlled by the agent - we don't want to include it in the returned states.

    @rtype: A list of ShipExternalState 
    @return: A list of estimated states of all other ships in the system
    """
    abstract

  def getEstimatedStatesOfShips(self, shipIndices):
    """
    Get a list of estimated ShipExternalState of all other ships
    indexed by shipIndices.

    @type  selfIndex: list[int]
    @param selfIndex: The indices of the ships for which location is estimated

    @rtype: A list of ShipExternalState 
    @return: A list of estimated states of all other ships in the system
    """
    abstract

  def getEstimatedExtShipState(self):
    """
    Get the estimated ShipExternalState as estimated
    by the world model.

    @rtype: ShipExternalState 
    @return: An estimation of the ship's real state according to our AgentWorldModel.
    """
    abstract

  def getEstimatedWindVector(self):
    """
    Get the estimated wind vector in the location of 
    the ship.

    @rtype: numpy array (vector)
    @return: an estimation of the wind vector (speed and direction) in the agent's location according to our AgentWorldModel.
    """
    abstract
    
  def getEstimatedWaterVector(self):
    """
    Get the estimated water vector in the location of 
    the ship.

    @rtype: numpy array (vector)
    @return: an estimation of the water vector (speed and direction) in the agent's location according to our AgentWorldModel.
    """
    abstract

  def getEstimatedObstaclesList(self):
    """
    Get a list of estimated obstacle shapes.

    @rtype: Obstacles
    @return: an estimation of the obstacles shapes (and positions) according to our AgentWorldModel.
    """
    abstract


  # non-abstract methods
  def cleanMsgQueue(self):
    self.incomingMsgs = CompositeMessage()

  def receiveMsg(self, msg):
    """
    Append a message to the queue
    """
    self.incomingMsgs.appendMsg(msg)
    # just a debug check
    if len(self.incomingMsgs.primitiveMsgs) > 100:
      raise Exception("Did you forget to read and clean messages")

  # TODO: more interface functions needed?


class AgentWorldModelCompleteWorld(AgentWorldModel):
  """
  Just a skeleton for testing purposes.
  The world model is exactly the world state.
  """
  def __init__(self, shipIndex):
    AgentWorldModel.__init__(self)
    self.state = None
    self.shipIndex = shipIndex

  # override
  def processPercepts(self, perceptsList):
    "Just search for the CompleteStatePercept and save the state from it"
    for percept in perceptsList.getPercepts():
      if isinstance(percept, perceptModels.CompleteStatePercept):
        # TODO: removed deepcopy because took time - but shallow copy should be fine
        #self.state = copy.deepcopy(percept.state)
        self.state = copy.copy(percept.state)
        return
    raise Exception("Couldn't find a CompleteStatePercept")

  def getEstimatedStatesOfOtherShips(self, selfIndex):
    """
    Implements the abstract method
    """
    states = self.state.shipExternalStates
    return states[0:selfIndex] + states[selfIndex + 1:]

  def getEstimatedStatesOfShips(self, shipIndices):
    """
    Implements the abstract method
    """
    states = self.state.shipExternalStates
    return [states[i] for i in shipIndices]

  def getEstimatedExtShipState(self):
    """
    Implements the abstract method
    """
    return self.state.shipExternalStates[self.shipIndex]

  def getEstimatedWindVector(self):
    """
    Implements the abstract method
    """
    shipState = self.state.shipExternalStates[self.shipIndex]
    posX, posY = shipState.x, shipState.y
    return self.state.sea.wind.getSpeedVectorInLocation(posX, posY)

  def getEstimatedWaterVector(self):
    """
    Implements the abstract method
    """
    shipState = self.state.shipExternalStates[self.shipIndex]
    posX, posY = shipState.x, shipState.y
    return self.state.sea.waterCurrents.getSpeedVectorInLocation(posX, posY)

  def getEstimatedObstaclesList(self):
    """
    Implements the abstract method
    """
    return self.state.sea.obstacles.getObstaclesList()


########################################################################
# Strategy class. Connecting between AgentWorldModel and the actions.
# A strategy is created for each pair of specific AgentWorldModel
# and a set of available actions (ship actuators).
# Note that for such a pair we can define more then one strategy,
# implementing different decision algorithms.
########################################################################
class AgentStrategy:
  """
  An interface for specific decision making strategy.
  It should be defined for a given AgentWorldModel, 
  a given set of legal actions, and a given decision making algorithm.

  For each step, the flow is:
  AgentWorldModel --> Specific_AgentStrategy --> Actions-to-execute
  """
  def __init__(self):
    # outgoingMsg is a message an agent (strategy) can send to
    # other agents and to the simulation environment
    self.outgoingMsg = CompositeMessage()

  def composeAction(self, agentWorldModel):
    "Given the (inferred) state of the world, decide on a set of actions."
    abstract

  def getOutgoingMessage(self):
    msg = self.outgoingMsg
    # reset if needed
    if len(msg.primitiveMsgs) > 0:
      self.outgoingMsg = CompositeMessage()
    return msg

  def transmit(self, msg):
    self.outgoingMsg.appendMsg(msg)


class AgentStaticPatrolStrategy(AgentStrategy):
  """
  Implements a basic patrol strategy.
  Given points of interest, just travel 
  along the cycle they define
  """
  def __init__(self, agentIndex, myPatrolPath, rulesOfTheSea):
    """
    Initializes the strategy.
      
    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents

    @type  path: a list of 2-tuples
    @param path: a list of (x,y) points that define a cycle for the agent to travel along

    @type  rulesOfTheSea: boolean
    @param rulesOfTheSea: Tells whether to respect the rules of the sea
    """
    AgentStrategy.__init__(self)

    self.agentIndex = agentIndex
    self.myPatrolPath = myPatrolPath
    self.rulesOfTheSea = rulesOfTheSea
    self.nextTaskPointIndex = 0

    self.navigationTactic = None

    self.time = 0
    self.prevPointTime = 0
    self.firstVisit = True

  def composeAction(self, agentWorldModel):
    """
    Implements the abstract method, see documentation 
    of the abstract method.
    """
    # if done, just stop for now
#    if self.nextTaskPointIndex == len(self.myPatrolPath):
#      action = CompositeAction()
#      action.appendAction(PrimitiveAction("stop", []))
#      return action
    self.time += 1

    if self.navigationTactic and self.navigationTactic.done(agentWorldModel):
      self.recordStatistics()

    self.setNextPointAndNavigationTacticIfNeeded(agentWorldModel)

    return self.navigationTactic.composeAction(agentWorldModel)

  def setNextPointAndNavigationTacticIfNeeded(self, agentWorldModel): 
    if self.navigationTactic == None:
      # i.e. first time we enter this function
      goalPos = self.myPatrolPath[self.nextTaskPointIndex]
      self.setNavigationTactic(agentWorldModel, goalPos)

    elif self.navigationTactic.done(agentWorldModel):
      self.nextTaskPointIndex = (self.nextTaskPointIndex + 1) % len(self.myPatrolPath)
      goalPos = self.myPatrolPath[self.nextTaskPointIndex]
      self.setNavigationTactic(agentWorldModel, goalPos)

  def setNavigationTactic(self, agentWorldModel, nextGoal):
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    # why round? because this value goes deep into 
    # rrtTree.findClosestNode and floating point arithmetic
    # significantly slows it down!
    estimatedPos = (int(round(estimatedState.x)), int(round(estimatedState.y)))

    if obstacleAvoidance.pathBlocked(estimatedPos, nextGoal, agentWorldModel.getEstimatedObstaclesList()):
      self.navigationTactic = AgentRRTTactic( self.agentIndex, 
                                              estimatedPos,
                                              nextGoal, 
                                              agentWorldModel.getEstimatedObstaclesList(), 
                                              self.rulesOfTheSea)
    else:
      self.navigationTactic = AgentPIDTactic(self.agentIndex,
                                             nextGoal,
                                             self.rulesOfTheSea)


  def getPath(self):
    return self.myPatrolPath


  def recordStatistics(self):
    # record results 
    if not self.firstVisit:
      prev = self.myPatrolPath[self.nextTaskPointIndex - 1]
      p = self.myPatrolPath[self.nextTaskPointIndex]

      # create an outgoing message
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_VISIT, 
                         p)
      )

      currentEdge = (prev, p)
      edgeLength = self.time - self.prevPointTime
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_EDGE_LENGTH, 
                         (currentEdge, edgeLength))
      )

    self.firstVisit = False
    self.prevPointTime = self.time


class AgentCoordinatedPatrolStrategy(AgentStrategy):
  """
  Implements a coordinated patrol strategy.
  Given points of interest, each agent
  runs heuristicDivide(), assign ships 
  to locations, and transmits it. Agents
  vote and decide on suggestion, move to
  their initial locations, and start 
  patroling.
  """

  # "Enum" for mission state
  TRANSMIT_INITIAL_LOCATION               = 1
  COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT = 2
  VOTE                                    = 3
  MOVING_TO_START                         = 4
  PATROLING                               = 5


  def __init__(self, agentIndex, patrolPoints, edgeLengths, rulesOfTheSea):
    """
    Initializes the strategy.
      
    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents

    @type  agentIndex: string
    @param agentIndex: the name of the file containing edges and nodes from measurements data

    @type  rulesOfTheSea: boolean
    @param rulesOfTheSea: Tells whether to respect the rules of the sea
    """
    AgentStrategy.__init__(self)

    self.agentIndex = agentIndex
    self.patrolPoints = patrolPoints
    self.edgeLengths = edgeLengths
    self.rulesOfTheSea = rulesOfTheSea
    # stats related vars
    self.time = 0
    self.prevPointTime = 0
    self.firstVisit = True
    # initialize coordinated-patrol related data members
    self.reset()

    # initializing the callbacks for processing messages
    self.msgCallbacks = {}
    self.msgCallbacks[PrimitiveMessage.TYPE_START_PATROL] = self.processMsgStart.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_POSITION] = self.processMsgPosition.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_PATROL_DIVISION] = self.processMsgPatrolDivision.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_REACHED_START] = self.processMsgReachedStart.__name__
    # .
    # .
    # .
    # continue adding callbacks


  def reset(self):
    """
    Resets all coordinated-patrol related data members
    """
    self.missionState = self.TRANSMIT_INITIAL_LOCATION
    self.shipPositions = {} # maps ships to their transmitted positions
    self.ship2patrolData = {} # would hold agent's suggestion
    self.ship2patrolDataSuggestions = []
    self.reachedStartPoint = set()
  
    self.myStartingPatrolPosition = None
    self.myPatrolPath = []
    self.nextTaskPointIndex = None
    self.navigationTactic = None
    #self.patrolStrategy = None


  def composeAction(self, agentWorldModel):
    # update time
    self.time += 1

    self.processIncomingMsgs(agentWorldModel)

    # by default, return a stopping action
    returnedAction = CompositeAction()
    returnedAction.appendAction(PrimitiveAction("stop", []))

    if self.missionState == self.TRANSMIT_INITIAL_LOCATION:
      myState = agentWorldModel.getEstimatedExtShipState()
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL, 
                         PrimitiveMessage.TYPE_POSITION, 
                         (agentWorldModel.shipIndex, myState.x, myState.y))
      )
      # move to the next state (TODO: assuming no msgs are lost)
      self.missionState = self.COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT


    elif self.missionState == self.COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT: 
      numShips = len(self.shipPositions)
      patrolPoints, paths, patrolStartingStates = heuristicDivide.run(numShips, 
          self.patrolPoints, self.edgeLengths, debug=False)
      state2ship = self.assignShipsToStates(self.shipPositions, patrolStartingStates)
      # build message
      self.ship2patrolData = {}
      for i, start_and_path in enumerate(zip(patrolStartingStates, paths)):
        start, path = start_and_path
        ship = state2ship[i]
        self.ship2patrolData[ship] = (start, path)
      msgContent = self.ship2patrolData
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL,
                         PrimitiveMessage.TYPE_PATROL_DIVISION,
                         msgContent)
      )
      # move to the next state 
      self.missionState = self.VOTE


    elif self.missionState == self.VOTE:
      for suggestion in self.ship2patrolDataSuggestions:
        if not suggestion == self.ship2patrolData:
          print 'suggestion', suggestion
          print 'mysuggestion', self.ship2patrolData
          raise Exception("vote failed - need to be implemented")
      # if we are here, everyone agrees
      self.myStartingPatrolPosition, self.myPatrolPath = self.ship2patrolData[self.agentIndex]
      self.missionState = self.MOVING_TO_START
      

    elif self.missionState == self.MOVING_TO_START:
      if self.navigationTactic == None:
        pos = self.myStartingPatrolPosition #just shorthand
        self.setNavigationTactic(agentWorldModel,
                                 (pos.x, pos.y))
        #self.navigationTactic = AgentPIDTactic(self.agentIndex,
        #                                       (pos.x, pos.y),
        #                                       self.rulesOfTheSea)
      # if reached target - transmit it
      if self.navigationTactic.done(agentWorldModel):
        msgContent = self.agentIndex
        self.transmit(
          PrimitiveMessage(PrimitiveMessage.TO_ALL,
                           PrimitiveMessage.TYPE_REACHED_START,
                           msgContent)
        )
        # this will restore to PID tactic (since already at goal), to wait
        # until all are ready
        pos = self.myStartingPatrolPosition #just shorthand
        self.setNavigationTactic(agentWorldModel,
                                 (pos.x, pos.y))
      returnedAction = self.navigationTactic.composeAction(agentWorldModel)
      if self.allReachedStartingPoints():
        print 'allReachedStartingPoints'
        self.missionState = self.PATROLING
        #initialize patrol points
        self.nextTaskPointIndex = 0
        self.setNavigationTactic(agentWorldModel, 
                                 self.myPatrolPath[self.nextTaskPointIndex])

    elif self.missionState == self.PATROLING:
      if self.navigationTactic.done(agentWorldModel):
        self.nextTaskPointIndex = (self.nextTaskPointIndex + 1) % len(self.myPatrolPath)
        self.setNavigationTactic(agentWorldModel, 
                                 self.myPatrolPath[self.nextTaskPointIndex])
        # also, record stats if segment is done
        self.recordStatistics()
        
      #if self.patrolStrategy == None:
      #  self.patrolStrategy = AgentStaticPatrolStrategy(self.agentIndex, 
      #                                                 self.myPatrolPath,
      #                                                 self.rulesOfTheSea)
      ## get action + msgs(!) from wrapped strategy, since
      ## these are the two outputs of composeAction()
      returnedAction = self.navigationTactic.composeAction(agentWorldModel)
      #for m in self.patrolStrategy.getOutgoingMessage().primitiveMsgs: self.transmit(m)

    return returnedAction

  def setNavigationTactic(self, agentWorldModel, nextGoal):
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    # why round? because this value goes deep into 
    # rrtTree.findClosestNode and floating point arithmetic
    # significantly slows it down
    estimatedPos = (int(round(estimatedState.x)), int(round(estimatedState.y)))

    if obstacleAvoidance.pathBlocked(estimatedPos, nextGoal, agentWorldModel.getEstimatedObstaclesList()):
      self.navigationTactic = AgentRRTTactic( self.agentIndex, 
                                              estimatedPos,
                                              nextGoal, 
                                              agentWorldModel.getEstimatedObstaclesList(), 
                                              self.rulesOfTheSea)
    else:
      self.navigationTactic = AgentPIDTactic(self.agentIndex,
                                             nextGoal,
                                             self.rulesOfTheSea)

  def recordStatistics(self):
    # record results
    if not self.firstVisit:
      prev = self.myPatrolPath[self.nextTaskPointIndex - 1]
      p = self.myPatrolPath[self.nextTaskPointIndex]

      # create an outgoing message
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_VISIT, 
                         p)
      )

      currentEdge = (prev, p)
      edgeLength = self.time - self.prevPointTime
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_EDGE_LENGTH, 
                         (currentEdge, edgeLength))
      )

    self.firstVisit = False
    self.prevPointTime = self.time

  def processIncomingMsgs(self, agentWorldModel):
    """
    Reads incoming msgs from the agentWorldModel
    and clean its queue

    @type  agentWorldModel: AgentWorldModel
    @param agentWorldModel: the world model, which contain msgs.
    """
    for msg in agentWorldModel.incomingMsgs.primitiveMsgs:
      # just a debug check
      if msg.to != PrimitiveMessage.TO_ALL:
        raise Exception("Message doesn't seem to be directed at me")
      # go to callback name, and convert to function using getattr
      getattr(self, self.msgCallbacks[msg.type])(msg.content)

  def assignShipsToStates(self, shipPositions, patrolStartingStates):
    """
    Implements a matching algorithm

    @type  shipPositions: map[shipIndex] = (x,y) position
    @param shipPositions: map of the transmitted positions of ships

    @type  patrolStartingStates: array of [shipExternalState, ... ] for all ships
    @param patrolStartingStates: the starting states computed by heuristicDivide

    @rtype:  map[shipIndex] = shipStartingState
    @return: mapping of each ship to its patrol starting state
    """
    numShips = len(shipPositions) 
    if not numShips == len(patrolStartingStates):
      raise Exception("number of ships must be equal to num patrol start points")

    # create a list of (ship, position, distance) tuples, sorted by distance
    edges = []
    for shipIndex, position in shipPositions.items():
      distances = [geometry.distance2D(position, (state.x, state.y)) for state in patrolStartingStates]
      edges += [(shipIndex, i, dist) for i, dist in enumerate(distances)]
    edges = sorted(edges, key=operator.itemgetter(2), reverse=True)

    # start removing distances until a node has only one edge, then create a pair
    # and remove all the other edges with the pair-mate
    shipNumEdges = {}
    posNumEdges = {}
    numOutgoingEdges = numShips
    for i in range(numOutgoingEdges):
      shipNumEdges[i] = numOutgoingEdges
      posNumEdges[i]  = numOutgoingEdges

    match = []
    matchedShips = set()     # for bookkeeping
    matchedPositions = set() # for bookkeeping
    # Heuristic match:
    # scan edges, throwing heaviest until a ship/node has
    # 1 outgoing edge, create a match, keep scanning. 
    # This process might not find a match for all, so 
    # repeat scans with left nodes/ships w/o a match
    while len(match) < numShips:
      deletedShips = set()
      deletedPositions = set()
      for edge in edges:
        shipIndex, posIndex, dist = edge
        # if we removed all the edges of a node - skip
        if shipIndex in deletedShips or posIndex in deletedPositions:
          continue

        if shipNumEdges[shipIndex] == 1 or posNumEdges[posIndex] == 1:
          match.append(edge)
          deletedShips.add(shipIndex)
          deletedPositions.add(posIndex)
          matchedShips.add(shipIndex)
          matchedPositions.add(posIndex)

        shipNumEdges[shipIndex] -= 1
        posNumEdges[posIndex] -= 1
      
      # remove edges for matched pairs before next round
      newEdges = []
      for edge in edges:
        shipIndex, posIndex, dist = edge
        if shipIndex in matchedShips or posIndex in matchedPositions:
          continue
        else:
          newEdges.append(edge)
      edges = newEdges

        

    state2Ship = {}
    for edge in match:
      shipIndex, posIndex, dist = edge
      state2Ship[posIndex] = shipIndex

    return state2Ship

  def allReachedStartingPoints(self):
    numShips = len(self.shipPositions)
    if len(self.reachedStartPoint) == numShips:
      return True
    else:
      return False

  def getPath(self):
    return self.myPatrolPath

  # msg processing callbacks
  def processMsgStart(self, msgContent): 
    """
    Processes a msg that notifies ships to start patrol

    @type  msgContent: None
    @param msgContent: no additional information currently needed
    """
    self.reset()

  def processMsgPosition(self, msgContent):
    """
    Processes a msg that notifies other ship's position

    @type  msgContent: a 3-tuple (shipIndex, x, y)
    @param msgContent: the position of the sending ship
    """
    shipIndex, x, y = msgContent
    self.shipPositions[shipIndex] = (x,y)

  def processMsgPatrolDivision(self, msgContent): 
    """
    Processes a msg that notifies other ship's patrol suggestion

    @type  msgContent: map[int] = (pos, list-of-positions)
    @param msgContent: for each ship, a starting position and a path
    """
    self.ship2patrolDataSuggestions.append(msgContent)

  def processMsgReachedStart(self, msgContent):
    """
    Processes a msg that notifies some ship reached its start pos

    @type  msgContent: int
    @param msgContent: an index of an agent that reached start
    """
    self.reachedStartPoint.add(msgContent)


class AgentJoiningPatrolStrategy(AgentCoordinatedPatrolStrategy):
  """
  Implements an agent that waits for some time, 
  then joins a coordinated patrol - this is why it is a coordinated
  patrol
  """

  def __init__(self, agentIndex, patrolPoints, edgeLengths, rulesOfTheSea):
    """
    Initializes the strategy.
      
    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents

    @type  agentIndex: string
    @param agentIndex: the name of the file containing edges and nodes from measurements data

    @type  rulesOfTheSea: boolean
    @param rulesOfTheSea: Tells whether to respect the rules of the sea
    """
    AgentCoordinatedPatrolStrategy.__init__(self, agentIndex, patrolPoints, edgeLengths, rulesOfTheSea)

    self.waitingTime = 1000
    self.currentTime = 0

  def composeAction(self, agentWorldModel):
    self.currentTime += 1
    if self.currentTime < 1000:
      returnedAction = CompositeAction()
      returnedAction.appendAction(PrimitiveAction("stop", []))

    elif self.currentTime == 1000:
      self.reset()
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL, 
                         PrimitiveMessage.TYPE_START_PATROL, 
                         None)
      )
      returnedAction = CompositeAction()
      returnedAction.appendAction(PrimitiveAction("stop", []))

    else:
      # call base class function
      returnedAction = AgentCoordinatedPatrolStrategy.composeAction(self, agentWorldModel)
    
    return returnedAction

    
class AgentTrackingPatrolStrategy(AgentStrategy):
  """
  Implements a basic patrol strategy.
  Given points of interest, just travel 
  along the cycle they define
  """

  # "Enum" for mission state
  TRANSMIT_LOCATION                       = 1
  COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT = 2
  VOTE_AND_SET_NAV_TARGET                 = 3
  #SET_NEW_TARGET                          = 4


  def __init__(self, agentIndex, trackedShipIndices, 
              trackingShipIndices, trackingDistance, positionOffsets,
              rulesOfTheSea):
    """
    Initializes the strategy.
      
    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents

    @type  trackedShipIndices: a list of integer indices
    @param trackedShipIndices: a list of indices of tracked ships

    @type  trackingShipIndices: a list of integer indices
    @param trackingShipIndices: a list of indices of tracking ships

    @type  trackingDistance: a floating point number
    @param trackingDistance: a (close to) maximal distance for tracking/sensing

    @type  positionOffsets: a list of numpy 2d vectors
    @param positionOffsets: offsets for all tracking ships  

    @type  rulesOfTheSea: boolean
    @param rulesOfTheSea: Tells whether to respect the rules of the sea
    """
    AgentStrategy.__init__(self)

    self.agentIndex = agentIndex
    self.trackedShipIndices = trackedShipIndices
    self.trackingShipIndices = trackingShipIndices
    self.trackingDistance = trackingDistance
    self.positionOffsets = positionOffsets
    self.rulesOfTheSea = rulesOfTheSea

    self.navigationTactic = None

    self.reset()

    self.missionState = self.TRANSMIT_LOCATION

    # initializing the callbacks for processing messages
    self.msgCallbacks = {}
    self.msgCallbacks[PrimitiveMessage.TYPE_TRACKING_POINT] = self.processMsgTrackingPoint.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_NAVIGATION_CENTRAL_POINT] = self.processMsgNavigationCentralPoint.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_POSITIONS_ASSIGNMENT] = self.processMsgPositionAssignments.__name__

    self.time = 0

  def reset(self):
    self.trackingPoints = {}
    self.navCentralPoint = {}
    self.ship2positionSuggestions = []

  def composeAction(self, agentWorldModel):
    """
    Implements the abstract method, see documentation 
    of the abstract method.
    """
    self.time += 1

    self.processIncomingMsgs(agentWorldModel)
    
    #if self.navigationTactic and self.navigationTactic.done(agentWorldModel):
    #  self.recordStatistics()

    self.setNextPointAndNavigationTacticIfNeeded(agentWorldModel)

    return self.navigationTactic.composeAction(agentWorldModel)

  def setNextPointAndNavigationTacticIfNeeded(self, agentWorldModel): 
    if self.navigationTactic == None:
      # initially create some tactic - to stand in place
      shipState = agentWorldModel.getEstimatedExtShipState()
      self.setNavigationTactic(agentWorldModel, (shipState.x, shipState.y))

    if self.missionState == self.TRANSMIT_LOCATION:
      self.trackingPoint = self.computedAvgPosOfTrackedShips(agentWorldModel)
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL, 
                         PrimitiveMessage.TYPE_TRACKING_POINT, 
                         self.trackingPoint)
      )
      self.navigationCentralPoint = self.computedAvgPosOfTrackingShips(agentWorldModel)
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL, 
                         PrimitiveMessage.TYPE_NAVIGATION_CENTRAL_POINT, 
                         self.navigationCentralPoint)
      )
      self.missionState = self.COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT

    elif self.missionState == self.COMPUTE_PATROL_ASSIGNMENTS_AND_TRANSMIT:
      goal = self.computeNextGoal(agentWorldModel) #numpy
      targets = self.generateNewTargets(goal, self.trackingPoint)
      trackingShipPositions = self.getTrackingShipLocations(agentWorldModel)
      self.ship2state = self.assignShipsToStates(trackingShipPositions, targets)
      msgContent = self.ship2state
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_ALL,
                         PrimitiveMessage.TYPE_POSITIONS_ASSIGNMENT,
                         msgContent)
      )
      # move to the next state 
      self.missionState = self.VOTE_AND_SET_NAV_TARGET
      
    elif self.missionState == self.VOTE_AND_SET_NAV_TARGET:
      for suggestion in self.ship2positionSuggestions:
        if not suggestion == self.ship2state:
          print 'suggestion', suggestion
          print 'mysuggestion', self.ship2state
          raise Exception("vote failed - need to be implemented")
      # if we are here, everyone agrees
      self.myStartingPatrolPosition = self.ship2state[self.agentIndex]
      goalPos = self.myStartingPatrolPosition
      self.setNavigationTactic(agentWorldModel, goalPos)
      # back to start
      self.missionState = self.TRANSMIT_LOCATION
      self.reset()

      
  def computeNextGoal(self, agentWorldModel):
    """
    Computes the next goal for the tracking strategy.
    """
    trackingPoint = self.computedAvgPosOfTrackedShips(agentWorldModel)
    navigationCentralPoint = self.computedAvgPosOfTrackingShips(agentWorldModel)
    distToTarget = geometry.distance2D(navigationCentralPoint, trackingPoint)
    newTrgt = navigationCentralPoint + (trackingPoint - navigationCentralPoint) * (1 - self.trackingDistance / distToTarget)
    return newTrgt

  def computedAvgPosOfTrackedShips(self, agentWorldModel):
    trackedShipPositions = self.getTrackedShipLocations(agentWorldModel).values()
    trackingPoint = geometry.averagePoint2D(trackedShipPositions)
    self.trackingPoint = trackingPoint # record for usage by GUI
    return trackingPoint

  def computedAvgPosOfTrackingShips(self, agentWorldModel):
    trackingShipPositions = self.getTrackingShipLocations(agentWorldModel).values()
    navigationCentralPoint = geometry.averagePoint2D(trackingShipPositions)
    return navigationCentralPoint 

  def setNavigationTactic(self, agentWorldModel, nextGoal):
    estimatedState = agentWorldModel.getEstimatedExtShipState()
    # why round? because this value goes deep into 
    # rrtTree.findClosestNode and floating point arithmetic
    # significantly slows it down
    estimatedPos = (int(round(estimatedState.x)), int(round(estimatedState.y)))

    if obstacleAvoidance.pathBlocked(estimatedPos, nextGoal, agentWorldModel.getEstimatedObstaclesList()):
      self.navigationTactic = AgentRRTTactic( self.agentIndex, 
                                              estimatedPos,
                                              nextGoal, 
                                              agentWorldModel.getEstimatedObstaclesList(), 
                                              self.rulesOfTheSea)
    else:
      self.navigationTactic = AgentPIDTactic(self.agentIndex,
                                             nextGoal,
                                             self.rulesOfTheSea)

  def getPath(self):
    return [self.navigationTactic.goalPoint]

  def recordStatistics(self):
    # record results 
    if not self.firstVisit:
      prev = self.myPatrolPath[self.nextTaskPointIndex - 1]
      p = self.myPatrolPath[self.nextTaskPointIndex]

      # create an outgoing message
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_VISIT, 
                         p)
      )

      currentEdge = (prev, p)
      edgeLength = self.time - self.prevPointTime
      self.transmit(
        PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
                         PrimitiveMessage.TYPE_EDGE_LENGTH, 
                         (currentEdge, edgeLength))
      )

    self.firstVisit = False
    self.prevPointTime = self.time

  def processIncomingMsgs(self, agentWorldModel):
    """
    Reads incoming msgs from the agentWorldModel
    and clean its queue

    @type  agentWorldModel: AgentWorldModel
    @param agentWorldModel: the world model, which contain msgs.
    """
    for msg in agentWorldModel.incomingMsgs.primitiveMsgs:
      # just a debug check
      if msg.to != PrimitiveMessage.TO_ALL:
        raise Exception("Message doesn't seem to be directed at me")
      # go to callback name, and convert to function using getattr
      getattr(self, self.msgCallbacks[msg.type])(msg.content)

  def processMsgTrackingPoint(self, msgContent):
    """
    Processes a msg that notifies the tracking point - avg location of tracked
    ships

    @type  msgContent: a tuple (int, numpy.array[(x,y)])
    @param msgContent: (suggestingShipIndex, suggestedTrackingPoint)
    """
    suggestingShipIndex, trackingPoint = msgContent
    self.trackingPoints[suggestingShipIndex] = trackingPoint

  def processMsgNavigationCentralPoint(self, msgContent):
    """
    Processes a msg that notifies the central navigation point - 
    the point from which ship offsets are computed

    @type  msgContent: a tuple (int, numpy.array[(x,y)])
    @param msgContent: (suggestingShipIndex, suggestedNavigationPoint)
    """
    suggestingShipIndex, navPoint = msgContent
    self.navCentralPoint[suggestingShipIndex] = navPoint 

  def processMsgPositionAssignments(self, msgContent): 
    """
    Processes a msg that notifies other ship's patrol suggestion

    @type  msgContent: map[int] = (pos)
    @param msgContent: for each ship, a target position
    """
    #self.ship2patrolDataSuggestions.append(msgContent)
    self.ship2positionSuggestions.append(msgContent)

  def assignShipsToStates(self, shipPositions, targets):
    """
    Implements a matching algorithm

    @type  shipPositions: map[shipIndex] = (x,y) position
    @param shipPositions: map of the transmitted positions of ships

    @type  targets: array of [(x,y), ... ] for all ships
    @param targets: the target locations for tracking

    @rtype:  map[shipIndex] = shipStartingState
    @return: mapping of each ship to its patrol starting state
    """
    numShips = len(shipPositions) 
    if not numShips == len(targets):
      raise Exception("number of ships must be equal to num patrol start points")

    # create a list of (ship, position, distance) tuples, sorted by distance
    edges = []
    for shipIndex, position in shipPositions.items():
      distances = [geometry.distance2D(position, target) for target in targets]
      edges += [(shipIndex, i, dist) for i, dist in enumerate(distances)]
    edges = sorted(edges, key=operator.itemgetter(2), reverse=True)

    # start removing distances until a node has only one edge, then create a pair
    # and remove all the other edges with the pair-mate
    shipNumEdges = {}
    posNumEdges = {}
    numOutgoingEdges = numShips
    for i in range(numOutgoingEdges):
      posNumEdges[i]  = numOutgoingEdges
    for shipIndex in shipPositions.keys():
      shipNumEdges[shipIndex] = numOutgoingEdges

    match = []
    matchedShips = set()     # for bookkeeping
    matchedPositions = set() # for bookkeeping
    # Heuristic match:
    # scan edges, throwing heaviest until a ship/node has
    # 1 outgoing edge, create a match, keep scanning. 
    # This process might not find a match for all, so 
    # repeat scans with left nodes/ships w/o a match
    while len(match) < numShips:
      deletedShips = set()
      deletedPositions = set()
      for edge in edges:
        shipIndex, posIndex, dist = edge
        # if we removed all the edges of a node - skip
        if shipIndex in deletedShips or posIndex in deletedPositions:
          continue

        if shipNumEdges[shipIndex] == 1 or posNumEdges[posIndex] == 1:
          match.append(edge)
          deletedShips.add(shipIndex)
          deletedPositions.add(posIndex)
          matchedShips.add(shipIndex)
          matchedPositions.add(posIndex)

        shipNumEdges[shipIndex] -= 1
        posNumEdges[posIndex] -= 1
      
      # remove edges for matched pairs before next round
      newEdges = []
      for edge in edges:
        shipIndex, posIndex, dist = edge
        if shipIndex in matchedShips or posIndex in matchedPositions:
          continue
        else:
          newEdges.append(edge)
      edges = newEdges

        
    ship2state = {}
    for edge in match:
      shipIndex, posIndex, dist = edge
      ship2state[shipIndex] = tuple(targets[posIndex])

    return ship2state

  def getTrackedShipLocations(self, agentWorldModel):
    """
    returns a list of x,y coordinates of tracked ships
    """
    states = dict( (i, (s.x, s.y)) for i,s in 
        zip(self.trackedShipIndices,
            agentWorldModel.getEstimatedStatesOfShips(self.trackedShipIndices) ) )
    return states

  def getTrackingShipLocations(self, agentWorldModel):
    """
    returns a list of x,y coordinates of tracking ships
    """
    states = dict( (i, (s.x, s.y)) for i,s in 
        zip(self.trackingShipIndices,
            agentWorldModel.getEstimatedStatesOfShips(self.trackingShipIndices) ) )
    return states

  def generateNewTargets(self, goal, trackingPoint):
    """
    Given the new goal and vector offsets - generate new targets.
    """
    # offsets are computed w.r.t. line to target
    direction = trackingPoint - goal
    direction = direction / numpy.linalg.norm(direction)
    perpendicular = numpy.dot(numpy.array([[0,1], [-1,0]]), direction)
    goal = numpy.array(goal)
    return [ goal + offset[0] * direction + offset[1] * perpendicular for offset in self.positionOffsets]


    
#   class AgentRandomPatrolStrategy(AgentStrategy):
#     """
#     Implements a basic patrol strategy.
#     Given points of interest, just travel 
#     along the cycle they define
#     """
#     def __init__(self, agentIndex, patrolArea, rulesOfTheSea):
#       """
#       Initializes the strategy.
#         
#       @type  agentIndex: int
#       @param agentIndex: sort of an agent-id - an agent's index in the array of all agents
#   
#       @type  rulesOfTheSea: boolean
#       @param rulesOfTheSea: Tells whether to respect the rules of the sea
#       """
#       AgentStrategy.__init__(self)
#   
#       self.agentIndex = agentIndex
#       self.patrolArea = patrolArea
#       self.rulesOfTheSea = rulesOfTheSea
#   
#     def composeAction(self, agentWorldModel):
#       """
#       Implements the abstract method, see documentation 
#       of the abstract method.
#       """
#       self.time += 1
#   
#       if self.navigationTactic and self.navigationTactic.done(agentWorldModel):
#         self.recordStatistics()
#   
#       self.setNextPointAndNavigationTacticIfNeeded(agentWorldModel)
#   
#       return self.navigationTactic.composeAction(agentWorldModel)
#   
#     def setNextPointAndNavigationTacticIfNeeded(self, agentWorldModel): 
#       if self.navigationTactic == None or self.navigationTactic.done(agentWorldModel):
#         # i.e. first time we enter this function
#         goalPos = self.getRandomGoal(self.patrolArea)
#         self.setNavigationTactic(agentWorldModel, goalPos)
#   
#     def setNavigationTactic(self, agentWorldModel, nextGoal):
#       estimatedState = agentWorldModel.getEstimatedExtShipState()
#       # why round? because this value goes deep into 
#       # rrtTree.findClosestNode and floating point arithmetic
#       # significantly slows it down!
#       estimatedPos = (int(round(estimatedState.x)), int(round(estimatedState.y)))
#   
#       if obstacleAvoidance.pathBlocked(estimatedPos, nextGoal, agentWorldModel.getEstimatedObstaclesList()):
#         self.navigationTactic = AgentRRTTactic( self.agentIndex, 
#                                                 estimatedPos,
#                                                 nextGoal, 
#                                                 agentWorldModel.getEstimatedObstaclesList(), 
#                                                 self.rulesOfTheSea)
#       else:
#         self.navigationTactic = AgentPIDTactic(self.agentIndex,
#                                                nextGoal,
#                                                self.rulesOfTheSea)
#   
#     def recordStatistics(self):
#       # record results 
#       if not self.firstVisit:
#         prev = self.myPatrolPath[self.nextTaskPointIndex - 1]
#         p = self.myPatrolPath[self.nextTaskPointIndex]
#   
#         # create an outgoing message
#         self.transmit(
#           PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
#                            PrimitiveMessage.TYPE_VISIT, 
#                            p)
#         )
#   
#         currentEdge = (prev, p)
#         edgeLength = self.time - self.prevPointTime
#         self.transmit(
#           PrimitiveMessage(PrimitiveMessage.TO_SIMULATOR, 
#                            PrimitiveMessage.TYPE_EDGE_LENGTH, 
#                            (currentEdge, edgeLength))
#         )
#   
#       self.firstVisit = False
#       self.prevPointTime = self.time
#   
#     def getRandomGoal(self, patrolArea):
#       p1, p2 = patrolArea
#       x1, y1 = p1
#       x2, y2 = p2
#       goalX = random.randint(x1, x2)
#       goalY = random.randint(y1, y2)
#       return (goalX, goalY)
#   


#class AgentCirclingStrategy(AgentStrategy):
#  """
#  NOTE: this class is not in use.
#  Implements a regular circling agent strategy
#  """
#
#  def __init__(self, speed):
#    AgentStrategy.__init__(self)
#
#    self.speed = speed
#
#  def composeAction(self, agentWorldModel):
#    action = CompositeAction()
#    action.appendAction(PrimitiveAction("setEngineSpeed", [self.speed]))
#    action.appendAction(PrimitiveAction("setSteering", [-1]))
#    return action
#  
#
#class CompleteStateHolonomicRRTSteeringEngineStrategy(AgentStrategy):
#  """
#  NOTE: this class is out-of-date and should be updated for the 
#  current environment and motion model (or actually to convert it into
#  non-holonomic, meaning to incorporate the sea physics constraints 
#  into the RRT planner)
#
#  A strategy for an RRT agent, where the world model is
#  a complete state and the set of legal actions are steering and 
#  engine speed. RRT is holonomic (i.e. assumes the agent has direct
#  control on the ship position), with no colision detection here.
#  The logical flow is:
#  complete-world-state model in the agent --> RRT --> Steering&Engine commands
#  """
#  def __init__(self, agentIndex, initialState, goalPos):
#    AgentStrategy.__init__(self)
#
#    self.agentIndex = agentIndex
#    self.goalPos = goalPos
#    self.actionIndex = 0
#
#
#
#
#    # TODO: Some hard-coded constants that should probably be sent as params
#
#    # Threshold to determine whether at goal
#    self.angleThreshold = 10
#    self.distanceThreshold = 10 
#
#    # borders for sampling
#    self.minX = 0
#    self.maxX = 1200
#    self.minY = 0
#    self.maxY = 800
#    self.minAng = 0
#    self.maxAng = 360
#
#    # step-fraction towards a new sampled state
#    self.epsilon = 0.1 
#
#
#
#    
#    # Compute the RRT path 
#    self.path = self.rrtPath(initialState, goalPos)
#
#
#
#  def composeAction(self, agentWorldModel):
#    "Implemeting the AgentStrategy interface."
#    # At the end, do the same action (which should be stop)
#    if self.actionIndex >= len(self.path):
#      return self.path[-1]
#
#    action = self.path[self.actionIndex]
#    self.actionIndex += 1
#    return action
#
#
#  def rrtPath(self, initialState, goalPos):
#    # create initial node
#    initialExternalState = copy.copy(initialState.shipExternalStates[self.agentIndex])
#
#    # init a tree   (TODO: change to realistic distanceUnit and angleUnit?)
#    self.tree = rrtTree.RRTTreeXYOrientationEuclidDist(initialExternalState, 
#                                                          distanceUnit=10, 
#                                                          angleUnit=5)
#
#    # search
#    currentState = initialExternalState
#    while not self.isGoalState(currentState):
#      currentState = self.extend(self.tree, self.chooseRandomState())
#
#    return self.tree.getPathTo(currentState)
#    
#
#  def extend(self, tree, randomState):
#    """
#    The EXTEND function from the paper.
#    Here, in this simple version of the algorithms, we just 
#    take an epsilon step towards the randomState, without checking
#    the all intermediate states are collision-free.
#    We also assume holonomic problem in which the computation
#    of the edge (the action that takes us to the new state) 
#    is just a simple vector calculation.
#
#    
#    Keyword arguments:
#    randomState -- A state that was randomly sampled in space, 
#                   towards which we take an epsilon step.
#
#    Returns:
#    The new state that was added to the tree.
#    
#    """
#    closestNode = tree.findClosestNode(randomState)
#    closestState = closestNode.state
#
#    # Take an epsilon step towards randomState
#    # TODO: need to normalize attributes with variance
#    closestStateX = closestState.x
#    closestStateY = closestState.y
#    closestStateOrient = closestState.orientation
#    newX = (randomState.x - closestStateX) * self.epsilon + closestStateX
#    newY = (randomState.y - closestStateY) * self.epsilon + closestStateY
#    newOrient = (randomState.orientation - closestStateOrient) * self.epsilon + closestStateOrient
#
#    # Compute the edge - the action to take in order to get to this state
#    # TODO: check that the path to this state is legal
#    xDiff = newX - closestStateX
#    yDiff = newY - closestStateY
#
#    if closestNode.parent != None:
#      oldOrientation = degrees( atan2(closestStateY - closestNode.parent.state.y,
#                                closestStateX - closestNode.parent.state.x) )
#    else:
#      oldOrientation = closestStateOrient
#    # Turn from the old direction to the new direction
#    steeringToApply = degrees( atan2(yDiff, xDiff) ) - oldOrientation 
#
#    speedToApply = sqrt( xDiff * xDiff + yDiff * yDiff )
#    action = CompositeAction()
#    action.appendAction(PrimitiveAction("setEngineSpeed", [speedToApply]))
#    action.appendAction(PrimitiveAction("setSteering", [steeringToApply]))
#
#    # Add the node to the tree
#    newState = state.ShipExternalState(newX, newY, newOrient, speedToApply, 0) 
#    parent = closestNode
#    edge = action
#    newNode = rrtTree.RRTNode(newState, parent, edge)
#    tree.addNode(newNode)
#
#    return newState
#    
#  def isGoalState(self,shipState):
#    "Is the ship within some threshold from the goal."
#
#    point1 = (shipState.x, shipState.y)
#    point2 = (self.goalPos.x, self.goalPos.y)
#    distance = geometry.distance2D(point1, point2)
#    if distance > self.distanceThreshold:
#      return False
#
#    # TODO: fix to deal with angles in a circular way,
#    # e.g. 359 and 1 are at distance 2 and not 358
#    angle1 = shipState.orientation
#    angle2 = self.goalPos.orientation
#    if abs(angle2-angle1) > self.angleThreshold:
#      return False
#
#    return True
#
#
#  def chooseRandomState(self):
#    "Samples a random state with a bias towards the goal."
#    if random.random() < 0.05:
#      return self.goalPos
#    else:
#      return state.ShipExternalState(random.randint(self.minX, self.maxX), 
#                                     random.randint(self.minY, self.maxY),
#                                     random.randint(self.minAng, self.maxAng),
#                                     speed = -1, angularSpeed = -1) # speed should be ignored
#
#      
#class MeasureEdgeLengths(AgentStrategy):
#  """
#  TODO: this class is for the regression learning 
#  currently not useful yet.
#  need to be made more generic when it becomes relevant.
#  A class that wraps some AgentStrategy, and adds
#  some measurements, to get experiment data.
#  The measurements it adds are the number of time steps it
#  took to reach the goal.
#  """
#  # TODO: this class is for the regression learning - need to be
#  #       made more generic when it becomes relevant.
#  def __init__(self, agentIndex, goalPos, agentStrategy):
#    AgentStrategy.__init__(self)
#
#    self.shipIndex = agentIndex
#    self.goalPos = goalPos
#    self.agentStrategy = agentStrategy
#    # Time counter
#    self.time = 0
#    # filename to append results
#    self.filename = "regression" + str(agentIndex)
#    # boolean - to write env data only in the first time
#    self.writtenEnvData = False
#
#  def composeAction(self, agentWorldModel):
#    # Increase time counter
#    self.time += 1
#
#    estimatedState = agentWorldModel.getEstimatedExtShipState(self.shipIndex)
#    estimatedPos = (estimatedState.x, estimatedState.y)
#
#
#    # For the first time write the conditions to params files
#    # TODO: hack, we only measure the environment at the beginning
#    if not self.writtenEnvData:
#      shipOrient = geometry.orientation2D(estimatedPos, self.goalPos)
#
#      windVector = agentWorldModel.getEstimatedWindVector(self.shipIndex)
#      windSpeed = geometry.distance2D((0,0), windVector)
#      windOrient = geometry.orientation2D((0,0), windVector)
#      print 'windOrient', windOrient
#      windRelOrient = geometry.computeShortestTurn(shipOrient, windOrient)
#
#      currentsVector = agentWorldModel.getEstimatedWaterVector(self.shipIndex)
#      waterSpeed = geometry.distance2D((0,0), currentsVector)
#      currentOrient = geometry.orientation2D((0,0), currentsVector)
#      print 'currentOrient', currentOrient
#      waterRelOrient = geometry.computeShortestTurn(shipOrient, currentOrient)
#
#      xFilename = self.filename + ".x"
#      if not os.path.exists(xFilename):
#        f = open(xFilename, 'w')
#        f.write('[')
#      else:
#        f = open(xFilename, 'r+')
#        f.seek(-1, 2)
#        f.write(', ')
#      f.write('(%.2f, %.2f, %.2f, %.2f)]' % ( windSpeed, 
#                                              windRelOrient, 
#                                              waterSpeed, 
#                                              waterRelOrient))
#      f.close()
#      self.writtenEnvData = True
#
#   
#    # If at goal - stop
#    distanceToGoal = geometry.distance2D(estimatedPos, self.goalPos)
#    if distanceToGoal <= 20:
#      yFilename = self.filename + ".y"
#      if not os.path.exists(yFilename):
#        f = open(yFilename, 'w')
#        f.write('[')
#      else:
#        f = open(yFilename, 'r+')
#        f.seek(-1, 2)
#        f.write(', ')
#      f.write(str(self.time) + ']')
#      f.close()
#      sys.exit()
#    else:
#      return self.agentStrategy.composeAction(agentWorldModel)

      






########################################################################
# The Agent class, encapsulates all the agent's functionality
########################################################################
class Agent:
  """
  This is a class for an agent that controls a ship.
  An agent has two main components: it's world model, 
  and its decision-making strategy.
  """
  def __init__(self, worldModel, strategy):
    self.worldModel = worldModel
    self.strategy = strategy

  def getAction(self, percepts):
    """
    A template method.
    Choose an action based on current percepts

    @type  percepts: PerceptList
    @param percepts: A list of percepts sent to the agent for this decision cycle

    @rtype: CompositeAction
    @return: A list of actions to be executed on the ship
    """
    self.worldModel.processPercepts(percepts)
    action = self.strategy.composeAction(self.worldModel)
    self.worldModel.cleanMsgQueue() # clean msgs, even if not processed
    return action

  def receiveMsg(self, msg):
    """
    msg is received by the agent using this function

    @type  msg: PrimitiveMessage
    @param msg: a primitive message sent to the agent.
    """
    # TODO: receiving messages should be part of the 
    # world model and not of the strategy - it seems like?
    return self.worldModel.receiveMsg(msg)

  def getOutgoingMessage(self):
    """
    Returns any message that strategy created

    @rtype: CompositeMessage
    @return: The agents' outgoing message for this decision cycle
    """
    return self.strategy.getOutgoingMessage()


      
