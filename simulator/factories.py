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



import random

########################################################################
# This file contains factory functions that are used in main.py
# to allocate objects based on cmd line options
########################################################################


###################################################
# AgentStrategy Factories.
# All should have an create(self, agentIndex) function
###################################################
#TODO: We might need to send specific data do different strategies.
#      This can be done using the **args method.
#      However, we don't do it yet as it's just an experimental mode, 
#      and anyway allocation method might be changed later

import agents
import state
class CirclingStrategyFactory:
  def create(self, agentIndex):
    return agents.AgentCirclingStrategy(speed=agentIndex)

class RRTStrategyFactory:
  def __init__(self, paths, rulesOfTheSea):
    self.paths = paths
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    agentPath = self.paths[agentIndex] # assuming path of length 2
    return agents.AgentRRTStrategy(agentIndex, agentPath[0], agentPath[1], self.rulesOfTheSea)
                          
class PIDStrategyFactory:
  def __init__(self, rulesOfTheSea):
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    return agents.AgentPIDStrategy(agentIndex, 
                                   (random.randint(0,1200), random.randint(0,600)), self.rulesOfTheSea)

#class MeasureEdgeLengthsFactory:
#  def create(self, agentIndex):
#    #TODO: this class is a hack, find better way to allocate and combine
#
#
#    ############## goalPos=(random.randint(0,1200), random.randint(0,600))
#    goalPos=(1100, 300)
#
#
#
#    agentStrategy = agents.AgentPIDStrategy(agentIndex, 
#                                                    goalPos)
#    return agents.MeasureEdgeLengths(agentIndex, goalPos, agentStrategy)


class AgentStaticPatrolStrategyFactory:
  def __init__(self, paths, rulesOfTheSea):
    self.paths = paths 
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    return agents.AgentStaticPatrolStrategy(agentIndex, self.paths[agentIndex], self.rulesOfTheSea)

class AgentCoordinatedPatrolStrategyFactory:
  def __init__(self, patrolPoints, edgeLengths, rulesOfTheSea):
    self.patrolPoints = patrolPoints 
    self.edgeLengths = edgeLengths 
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    return agents.AgentCoordinatedPatrolStrategy(agentIndex, self.patrolPoints, self.edgeLengths, self.rulesOfTheSea)

class AgentJoiningPatrolStrategyFactory:
  def __init__(self, patrolPoints, edgeLengths, rulesOfTheSea):
    self.patrolPoints = patrolPoints 
    self.edgeLengths = edgeLengths 
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    return agents.AgentJoiningPatrolStrategy(agentIndex, self.patrolPoints, self.edgeLengths, self.rulesOfTheSea)


class AgentTrackingPatrolStrategyFactory:
  def __init__(self, trackedShipIndices, trackingShipIndices, trackingDistance,
      positionOffsets, rulesOfTheSea):
    self.trackedShipIndices = trackedShipIndices 
    self.trackingShipIndices = trackingShipIndices 
    self.trackingDistance = trackingDistance
    self.positionOffsets = positionOffsets
    self.rulesOfTheSea = rulesOfTheSea

  def create(self, agentIndex):
    return agents.AgentTrackingPatrolStrategy(agentIndex, 
        self.trackedShipIndices, self.trackingShipIndices,
            self.trackingDistance, self.positionOffsets, self.rulesOfTheSea)

#   class AgentRandomPatrolStrategyFactory:
#     def __init__(self, patrolArea, rulesOfTheSea):
#       self.patrolArea = patrolArea 
#       self.rulesOfTheSea = rulesOfTheSea
#   
#     def create(self, agentIndex):
#       return agents.AgentRandomPatrolStrategy(agentIndex, self.patrolArea, self.rulesOfTheSea)



###################################################
# AgentWorldModel Factories.
# All should have an create(self) function
###################################################
import agents
class CompleteWorldModelFactory:
  def create(self, shipIndex): 
    return agents.AgentWorldModelCompleteWorld(shipIndex)




###################################################
# Ship Factories.
# All should have an create(self) function
###################################################
import shipModels
class BasicShipFactory:
  def create(self): 
    return shipModels.Ship()
