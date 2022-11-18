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



from math import *
import numpy
import random

from messages import PrimitiveMessage

class SimulationEnvironment:
  """

  Based on AIMA's environment:
  Abstract class representing an Environment.  
  'Real' Environment classes inherit from this. 
  Your Environment will typically need to implement:
  percept:           Define the percept that an agent sees.
  executeAction:    Define the effects of executing an action.

  """

  def __init__( self, initialState, shipAgents ):
    """
    Initialize the simulation environment.

    @type  initialState: State
    @param initialState: The initial state of the sea and ships

    @type  shipAgents: array of Agents
    @param shipAgents: The initial state of the sea and ships
    """
    self.state = initialState
    self.shipAgents = shipAgents
    # initializing the callbacks for processing messages
    self.msgCallbacks = {}
    self.msgCallbacks[PrimitiveMessage.TYPE_VISIT] = self.processMsgVisit.__name__
    self.msgCallbacks[PrimitiveMessage.TYPE_EDGE_LENGTH] = self.processMsgEdgeLength.__name__
    # .
    # .
    # .
    # continue adding callbacks

    # different data fields that gather some data about the simulation
    self.time = 0 # simulation time: 1, 2, ...   (logically, 0 is the initial state so couting from 1)
    self.visitTimes = {} # maps a point to a list of times it has been visited
    self.edgeLengths = {} # maps an edge to a list of measured edge travel-times
    


  def percept(self, agentIndex):
    """
    Return the percept that the agent sees at this point.
    We view the world such that the complete state is sent to 
    a ship, and a ship can only extract from it what it can percept.
    """
    return self.state.ships[agentIndex].getPercepts(self.state)

  def executeAction(self, agentIndex, action):
    """Change the world to reflect this action.
    Here, we only change the ship's internal state.
    The external state will be changed in exogenousChange()

    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents

    @type  action: CompositeAction
    @param action: An action chosen by the agent, to be executed on the ship
    """ 
    self.state.ships[agentIndex].executeAction(action)

  def exogenousChange(self):
    """
    Compute a change in the world state, that is external to the agent.
    This function is called in every simulation step, after computing the
    effects of the agent's actions.

    Here the change that is computed is:
      - A change in the ships' external states, which are outside the control
         of the agent. (Ships' internal states are changed by agent actions.)
      - A change in the sea conditions
    """
    # 2 steps: ship state change, and then 
    # enviroment change as a result of time
    for i in range(len(self.state.ships)):
      self.updateShipExternalState(i)

    # TODO: put other value for time(?)
    self.computeChangeInSeaConditions(timePassed=1)

  def step(self):
    """
    Run the SimulationEnvironment for one time step (One second).

    Clarification, originally from the AIMA book's code:
    If the actions and exogenous changes are independent, 
    this method will do.  
    If there are interactions between them, you'll need to
    override this method.
    """
    self.time += 1

    if not self.isDone():
      actions = [agent.getAction(self.percept(agentIndex)) for agentIndex, agent in enumerate(self.shipAgents)]
      for (agentIndex, action) in enumerate(actions):
        self.executeAction(agentIndex, action)
      self.exogenousChange()

      # TODO: currently messages are not part of an "action" - 
      #       they are a kind of "metadata" that is being communicated. 
      #       If needed, this could be changed in the future.
      self.processAgentsMessages()

    # ORIGINAL CODE
    '''
    if not self.isDone():
      actions = [agent.getAction(self.percept(agent)) for agent in self.shipAgents]
      for (agentIndex, action) in enumerate(actions):
        self.executeAction(agentIndex, action)
      self.exogenousChange()
    '''


  def isDone(self):
    "By default, we're done when we can't find a live agent."
#    for agent in self.agents:
#      if agent.is_alive(): return False
#    return True
    return False

  def run(self, steps=250000):
    """Run the SimulationEnvironment for given number of time steps."""
    for step in xrange(steps): # xrange is a generator, more efficient
      if self.isDone(): 
        return
      self.step()
    print 'Done!'

  def updateShipExternalState(self, shipIndex):
    """
    Compute ship position change in time, given the current state.

    Computation of the next state is based on the world state, 
    the ship internal and external state and time passed.

    Currently, we approximate ship movement using the following model: 
      - For forward motion, we model forward force that operates on the ship 
         by the engine, and a drag, which is quadratic in the ship's speed.
      - For turning, there is an rotational torque that is applied by the rudder, 
         and is proportional to the ship's speed, and to sin(rudder-steering-angle),
         which is the projection of the rudder on the lateral direction.
         There is also a rotational drag force, that is quadratic in the ship's 
         angular speed, (need to check about the accuracy of this modelling). 
      - Based on the above forces and the ship's mass, we compute the 
         forward and angular accelerations.
      - Then based on the average forward and angular speed in a given time step, 
         computed using the above accelerations, we infer the turn radius, 
         and based on that, compute the ship position at the end of this time-step. 

    Approximations we make:

      - Although the angular acceleration depends on forward speed, 
         which is changing due to forward acceleration, we still 
         assume constant angular acceleration, based on the avg. speed in this step.
      - Forward acceleration computation does not take into account the affects 
         of turning which (might?) slow it down.
      - Forward acceleration depends on the drag, which is changing with speed change,
         but we approximate the drag based on the initial speed of a time step
    
    """
    
    # TODO: later stage: collisions?

    # Get the needed data
    ship = self.state.ships[shipIndex]
    s = self.state.shipExternalStates[shipIndex]
    shipSpeed = s.speed
    angularSpeed = s.angularSpeed
    timeStepLength = 1 # TODO: change that in the future


    # Compute the front acceleration and avg speed
    forwardForce = ship.getForwardForce(shipSpeed)
    forwardAcceleration = ship.getForwardAccelerationFromForce(forwardForce)
#    print 'forwardAcceleration', forwardAcceleration
    # v = v0 + at  => avg = (v0 + v) / 2 = (v0 + v0 + at) / 2 = v0 + at/2
    avgSpeed = shipSpeed + forwardAcceleration * timeStepLength * 0.5
#    print 'avgSpeed', avgSpeed

    # Compute the angular acceleration and avg speed
    steeringForce = ship.getSteeringForce(avgSpeed, angularSpeed)
    angularAcceleration = ship.getAngularAccelerationFromForce(steeringForce)
#    print 'angularAcceleration', angularAcceleration
    avgAngularSpeed = angularSpeed + angularAcceleration * timeStepLength * 0.5
#    print 'avgAngularSpeed', avgAngularSpeed
    avgAngularSpeedRad = radians(avgAngularSpeed)
#    print 'avgAngularSpeedRad', avgAngularSpeedRad


    # Compute the new position at the end of this time step 
    # TODO: This is an approximation. We compute the avg speed and the avg angular
    # speed, and using a geometric computation for motion in constant 
    # forward / angular speeds.
    
    if avgAngularSpeedRad == 0: # going straight
      relXTranslation = avgSpeed * timeStepLength
      relYTranslation = 0
    else:
      radius = float(avgSpeed) / avgAngularSpeedRad # the 0.0 is for floating point division
      relXTranslation = radius * sin(avgAngularSpeedRad * timeStepLength) # yes, it's sine...
      relYTranslation = radius * (1 - cos(avgAngularSpeedRad * timeStepLength)) # 
      # A fix for turning in negative direction
      if avgAngularSpeedRad < 0:
        relYTranslation = -relYTranslation
#    print 'relXTranslation', relXTranslation, 'relYTranslation', relYTranslation

    startAngle = s.orientation
#    print 'startAngle', startAngle
    cos_start_angle = cos(radians(startAngle))
    sin_start_angle = sin(radians(startAngle))
    frontDirectionVector = numpy.array( [cos_start_angle , sin_start_angle] )
#    print 'frontDirectionVector', frontDirectionVector
    sideDirectionVector = numpy.array( [-sin_start_angle, cos_start_angle] )
#    print 'sideDirectionVector', sideDirectionVector
    shipPos = numpy.array( [s.x, s.y] )
#    print 'shipPos', shipPos
    shipNewPos = shipPos + relXTranslation * frontDirectionVector + relYTranslation * sideDirectionVector
#    print 'shipNewPos', shipNewPos

    # Offset caused by environment conditions
    sea = self.state.sea
    shipNewPos += ship.getOffsetByEnvConditions(sea.wind.getSpeedVectorInLocation(s.x, s.y),
                                                sea.waterCurrents.getSpeedVectorInLocation(s.x, s.y),
                                                startAngle)
    # TODO: for DEBUG
    progressVector = shipNewPos - shipPos
#    print 'progressAngle', degrees(atan2(progressVector[1], progressVector[0]))

    s.x = shipNewPos[0] #+ random.gauss(0, 2)
    s.y = shipNewPos[1] #+ random.gauss(0, 2)
    s.orientation += avgAngularSpeed * timeStepLength #+ random.gauss(0, 2)
    # normalize (according to the range of atan2() )
    while( s.orientation > 180 ):
      s.orientation -= 360
    while( s.orientation <= -180 ):
      s.orientation +=360
    s.speed += forwardAcceleration * timeStepLength
    s.angularSpeed += angularAcceleration * timeStepLength

#    print 'new state', s.x, s.y, s.orientation, s.speed, s.angularSpeed
#    print

    return 

#    # Original code that is simple but not realistic movement
#    ship = self.state.ships[shipIndex]
#    s = self.state.shipExternalStates[shipIndex]
#    speed = ship.engineSpeed# TODO: Incorrect, change!
#    s.orientation += ship.steering # TODO: Incorrect, change!
#    xTranslation = speed * cos(radians(s.orientation))
#    yTranslation = speed * sin(radians(s.orientation)) 
#    # TODO: does it affect the original state?
#    s.x += xTranslation
#    s.y += yTranslation

  def computeChangeInSeaConditions(self, timePassed):
    "Compute sea state change in time"
    # TODO: implement
    pass



  #####################################
  # COMMUNICATION MODULE
  #####################################
  def processAgentsMessages(self):
    """
    The communication module of the simulator.
    Each round, all agents' messages are being processed.
    """
    for agentIndex, agent in enumerate(self.shipAgents):
      compositeMsg = agent.getOutgoingMessage()
      self.processCompositeMessage(compositeMsg, agentIndex)

  def processCompositeMessage(self, compositeMsg, agentIndex):
    """
    Processes a Composite messages by breaking it into primitive messages
    and using the appropriate callbacks based on the message type.

    @type  compositeMsg: compositeMsg
    @param compositeMsg: Contains 0 or more primitive messages from one agent.

    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents
    """
    for msg in compositeMsg.primitiveMsgs:
      if msg.to == PrimitiveMessage.TO_SIMULATOR:
        # go to callback name, and convert to function using getattr
        getattr(self, self.msgCallbacks[msg.type])(msg, agentIndex)

      elif msg.to == PrimitiveMessage.TO_ALL:
        for agent in self.shipAgents:
          agent.receiveMsg(msg)

      else:
        raise Exception("Unknown message receipient")


  def processMsgVisit(self, msg, agentIndex):
    """Processes a msg that notifies about an agent visit at a point.
    
    @type  msg: messages.PrimitiveMessage
    @param msg: a message with a specifically assumed format (see below)

    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents
    """
    # format is assumed to be "(x,y)" so we just evaluate it
    point = msg.content

    # Note: here we don't use the "to" field of the msg, 
    #       as it's assumed to be sent to the environment

    # just a regular dictionary assignment visitTimes[point].append(self.time)
    # that automatically takes care of initialization. 
    self.visitTimes.setdefault(point, []).append(self.time)

    #print 'processMsgVisit', point, self.time
      
  def processMsgEdgeLength(self, msg, agentIndex):
    """
    Processes a msg that notifies the length of 
    an edge that an agent just completed.
    
    @type  msg: messages.PrimitiveMessage
    @param msg: a message with a specifically assumed format (see below)

    @type  agentIndex: int
    @param agentIndex: sort of an agent-id - an agent's index in the array of all agents
    """
    # format is assumed to be "(((x1,y1),(x2,y2)), length)" so we just evaluate it
    edge, length = msg.content

    # Note: here we don't use the "to" field of the msg, 
    #       as it's assumed to be sent to the environment

    # just a regular dictionary assignment visitTimes[point].append(self.time)
    # that automatically takes care of initialization. 
    self.edgeLengths.setdefault(edge, []).append(length)

    #print 'processMsgEdgeLength', edge, length

