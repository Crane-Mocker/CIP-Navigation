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



#polarsToPotentialObstacles()
#haveFreeCorridor()
#computeCorridor()
#polarsToObstaclesInsideCorridor()
from vec2d import *
from lineline import *

def pathBlocked(curPos, nextPos, obstacleList):
  for obst in obstacleList:
    A = Vec2d(curPos)
    B = Vec2d(nextPos)
    C, D = obst.getBorder()
    C = Vec2d(C)
    D = Vec2d(D)

    if lineline(A, B, C, D):
      #intersects!
      return True
  return False



# # We decided not to use the below (incompleted) function for now, as we already have the RRT

# def avoidObstacles(estimatedState, goalPos, estimatedObstaclesList): 
#   """
#   Given a distance and angle to the desired goal, and a list 
#   of estimated obstacles, return a new distance and angle
#   to a temporary target on a path that avoids the current target.
# 
#   @type  estimated: ShipExternalState
#   @param estimated: estimated state of the ship
# 
#   @type  goalPos: (x, y) tuple of doubles
#   @param goalPos: goal position of the ship
# 
#   @rtype  : a pair of doubles 
#   @return : a new goal (x, y) 
#   """
#   # Implement the following decision scheme
#   # If we are about to collide: if possible back up, otherwise stop.
#   # Else, do we have a "corridor" to target?
#   # ->Yes: GO
#   # ->No: can deviation help? (i.e. is corridor narrowed from one side only?)
#   #       ->Yes: deviate
#   #       ->No: there is a block - look right and left and take the smallest deviation that would bring you to the edge
# 
#   # TODO: this should be a parameter?
#   SAFETY_RADIUS = 5 # ship doesn't get closer than that to obstacles
# 
#   # if about to collide stop
#   # TODO: implement
# 
#   # do we have a corridor?
#   polarsToCheck = polarsToPotentialObstacles(estimatedState, 
#                                              goalPos, 
#                                              estimatedObstaclesList)
#   corridor = computeCorridor(estimatedState, goalPos, SAFETY_RADIUS)
#   polarsInsideCorridor = polarsToObstaclesInsideCorridor(polarsToCheck, 
#                                                          corridor)
#   if haveFreeCorridor(polarsInsideCorridor):
#     # no avoidance action is needed
#     return goalPos
# 
#   # else blocked, check in what way


