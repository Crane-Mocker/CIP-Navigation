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




class ShipExternalState:
  """
  The state of the ship for an outside observer
  """

  def __init__(self, x, y, orientation, speed=0, angularSpeed=0):
    """
    Initialize an I{external state} of a ship, namely its position and speeds

    @type  x: float
    @param x: x-position, in meters

    @type  y: float
    @param y: y-position, in meters

    @type  orientation: float
    @param orientation: the ship orientation, where -180 < orientation <= 180 degrees

    @type  speed: float
    @param speed: ship forward speed in m/sec (TODO: need to replace it by x-speed and y-speed)

    @type  angularSpeed: float
    @param angularSpeed: ship angular speed in deg / sec
    """
    self.x = x
    self.y = y
    self.orientation = orientation
    self.speed = speed
    self.angularSpeed = angularSpeed

  def __eq__(self, other):
    return (isinstance(other, self.__class__) and self.__dict__ == other.__dict__)

  def __str__(self):
    return "\nExternal State:" + \
           " x=" + str(self.x) + \
           " y=" + str(self.y) + \
           " orientation=" + str(self.orientation) + \
           " speed=" + str(self.speed) + \
           " angularSpeed=" + str(self.angularSpeed) + "\n"

    
    


class State:
  """
  This is a base class for the world state.
  """

  def __init__(self, sea, ships, shipExternalStates):
    """
    Initialize the world state that is external to the agent.

    @type  sea: Sea
    @param sea: the sea model

    @type  ships: array of Ship
    @param ships: the ships being simulated

    @type  shipExternalStates: array of ShipExternalStates
    @param shipExternalStates: the external states of the ships, namely their global positions
    """
    #print 'Creating world state'
    self.sea = sea
    self.ships = ships
    self.shipExternalStates = shipExternalStates

  def __str__(self):
    res = "\n\nWorld State: \n"
    res += str(self.sea)
    res += "\nShips:\n"
    for ship, state in zip(self.ships, self.shipExternalStates):
      res += str(ship) + str(state)
    return res

#  def generateSuccessorFromTime( self, timePassed ):
#    """
#    State change due to time passing
#    """
#
#    # TODO: is function needed?
#
#    util.raiseNotDefined()
#    return self
#
#
#  def generateSuccessorFromAction( self, action, shipIndex ):
#    """
#    State change due to ship action
#    """
#    
#    # TODO: is this function needed?
#
#    util.raiseNotDefined()
#    self.ships[shipIndex].applyAction( action )
#
#    return self
#
