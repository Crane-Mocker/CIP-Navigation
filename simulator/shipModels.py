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

from perceptModels import *

###############################################################
# PerceptionModule - the perception part of the ship
###############################################################
class PerceptionModule:
  """
  Interface for the perception system of a ship.
  """

  def getListOfPercepts(self, fullState):
    """
    An abstract function, that defines an interface for perception. 
    Its input is the full world state, and its output is a list
    of percepts that the ships senses, based on the current world
    state.
    """
    abstract


class CompleteStatePerceptionModule(PerceptionModule):
  "Percepts the complete state of the world"

  def getListOfPercepts(self, fullState):
    res = PerceptList()
    res.appendPercept(CompleteStatePercept(fullState))
    return res



######################################################
# The Ship class - modeling all aspects of a ship
######################################################

class Ship:
  """
  This class describes a general ship.
  A ship has:
    - Perception capability
    - A set of legal actions that can be applied to it, through its interface functions.
    - An internal state (currently the state of it's actuators)
    that is controlled through the set of legal actions.
    - Functions that describes its response to the environment, and depends on its
  physical properties.

  Any implementation of any of the above part can be plugged-in, allowing
  to mix different combinations of capabilities into a ship.

  A ship's external state, which is its coordinates, velocity, orientation
  and so on, is not part of the ship itself, but part of the environment,
  and is part of the world state.
  """
  # TODO: later abstract a ship to a percetion module, and action module
  #       that define what can the ship percept, and what actions
  #       can be done on it? and also allow to mix those in different ships
  def __init__(self, perceptionModule = CompleteStatePerceptionModule(),
                     initialEngineSpeed = 0, 
                     initialSteering = 0, 
                     maxSpeed = 40,
                     minSpeed = -10,
                     maxSteering = 90,
                     minSteering = -90,
                     mass = 1000,
                     #mass = 100,
                     length = 12):
    #print 'Creating ship model'
    self.perception = perceptionModule
    self.engineSpeed = initialEngineSpeed
    self.steering = initialSteering
    self.maxSpeed = maxSpeed
    self.minSpeed = minSpeed
    self.maxSteering = maxSteering
    self.minSteering = minSteering
    self.mass = mass
    self.length = length

  def __str__(self):
    res = "\nShip:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    return res


  #############################################
  # Functions supplied for the agent
  #############################################

  def getPercepts(self, fullState):
    """
    We view the ship as receiving a full world state
    and then "filtering" it to only the percepts it is
    able to process. Philosophically, we could view
    the world that way: we are only able to process 
    part of the complete information that surrounds us.
    """
    return self.perception.getListOfPercepts(fullState)

  def executeAction(self, action):
    """
    Change the internal state of a ship as a result of an action
    that was chosen by the ship agent.
    The 'action' argument can be a (possibly empty) composite action, 
    and in general is a list of pairs of the form (methodname, args), 
    such that self.methodname(args) is executed for each pair.
    """
    for a in action.primitiveActions:
      getattr(self, a.functionName) (*a.args)


  ###########################################
  # Physical actions a ship should provide
  ###########################################
  
  def setSteering(self, steering):
    if steering < self.minSteering or steering > self.maxSteering:
      print '-W- steering is out of range, action has no affect'
      return 
      print '-W- steering is out of range, fix that'
    self.steering = steering


  def setEngineSpeed(self, speed):
    if speed < self.minSpeed or speed > self.maxSpeed:
      print '-W- speed is out of range, action has no affect'
      return 
      print '-W- speed is out of range, fix that'
    self.engineSpeed = speed

  def stop(self):
    """Stop the ship.
    This function might need to be changed because it will not always be
    correct that setting everything to 0 will stop it
    """
    self.setSteering(0)
    self.setEngineSpeed(0)


  ###########################################
  # Physical response to the environment 
  # (TODO: in the future, extract this part to a seperate class, for example like the perception module)
  ###########################################
  def getForwardForce(self, shipSpeed):
    """
    Computing the forward force applied on the ship based on the engine and the drag forces.
    """
    forwardDragConstant = 100.0 # some proportionality constant
    # TODO: change engineSpeed
    engineForce = self.engineSpeed * 50.0
    # Drag is proportional to speed^2, with direction opposite to the speed
    dragForce = -shipSpeed * abs(shipSpeed) * forwardDragConstant
#    print 'forwardForce', engineForce + dragForce, 'engineForce', engineForce, 'forwardDragForce', dragForce, 'shipSpeed', shipSpeed
    return engineForce + dragForce

  def getSteeringForce(self, shipSpeed, shipAngularSpeed):
    """Computing the steering force based on the, the speed, the drag and so on."""
    # TODO: fix according to the comment above
    const = 1.5 
    angularDragConst = 0.5
    rudderForce = sin(radians(self.steering)) * shipSpeed * const
    # Drag is proportional to speed^2, with direction opposite to the speed
    dragForce = -shipAngularSpeed * abs(shipAngularSpeed) * angularDragConst
#    print 'rudderForce', rudderForce, 'dragForce', dragForce, 'steering', self.steering, 'shipSpeed', shipSpeed, 'shipAngularSpeed', shipAngularSpeed
    return rudderForce + dragForce

  def getForwardAccelerationFromForce(self, forwardForce):
    return forwardForce / self.mass

  def getAngularAccelerationFromForce(self, steeringForce):
    """
    Assuming mass is spread homogenously.
    Using formulas from http://rockpile.phys.virginia.edu/arch16.pdf
    that relates angular acceleration to torque.
    """
    radius = self.length / 2.0
    return degrees( steeringForce / ( 1 / 12.0 * self.mass * radius) )

  def getOffsetByEnvConditions(self, windVector, currentVector, orient):
    """
    Compute a ship's response to the environment conditions.
    
    @type  windVector: NumPy vector
    @param windVector: vector of the wind in the ship's location

    @type  currentVector: a numpy vector
    @param currentVector: vector of the current in the ship's location

    @type  orient: float
    @param orient: The ship's orientation (part of determining the response)

    @rtype: NumPy vector
    @return: An (x, y) offset of the ship, as a numpy vector
    """
    # TODO: currently we don't use the orient parameter - change that.
    windConstant = 0.01
    currentConstant = 0.1
    return windConstant * windVector + currentConstant * currentVector



