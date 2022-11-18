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



#################################################################################
# This file defines three variables loaded into the simulator, 
# desribing the following three variables. 
# 1.  patrolPoints: 
#     ~~~~~~~~~~~~
#     - This is a list of all possible (x,y) points that 
#       would ever be travelled by any ship.
#     - The variable type is a list
#     - For example: 
#       patrolPoints = [(x0,y0), (x1,y1), ...]
#  
# 2.  paths:
#     ~~~~~
#     - This is a list of patrol-paths, one patrol path for each ship.
#     - The variable type is a list of paths, where each path is a 
#       list of (x,y) points, taken from patrolPoints.
#     - For example:
#       p = patrolPoints
#       paths = [ [p[0], p[5], p[3]], [p[3], p[7], p[11]], ... ]
#      
# 3.  shipExternalStates:
#     ~~~~~~~~~~~~~~~~~~~
#     - This is a list of the initial states of the ships. 
#     - The variable type is a list of 5-tuples, where each tuple represents a state.
#       A state's 5-tuple is (x, y, orientation, speed, angular-speed). 
#     - For example:
#       s = [ [(100, 200, 0, 0, 0), (800, 400, 0, 0, 0), ... ]
#       shipExternalStates = [state.ShipExternalState(*s[i]) for i in range(numShips)]
#     
#     
# Note: As this is a python file, it is possible to both insert data in the 
# simple format of the basic example, or using more sophisticated python commands.
#     
#####################################################################################




##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#

# POINTS FOR THE BASIC EXAMPLE
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
patrolPoints = [(300,500), (750,290), (1090,655), (1390,655), (1866,505), (1866,995), (1390,845), (1090,845), (750,1210), (300,1000)]

path1 = patrolPoints
path2 = list(patrolPoints); path2.reverse()
path3 = [patrolPoints[2], patrolPoints[3], patrolPoints[6], patrolPoints[7]]
path4 = [patrolPoints[7], patrolPoints[6], patrolPoints[3], patrolPoints[2]]
paths = [path1, path2, path3, path4]
#paths =[ [(300,500), (750,290), (1090,655), (1390,655), (1866,505), (1866,995), (1390,845), (1090,845), (750,1210), (300,1000)],
#              [(1090,845), (750,1210), (300,1000), (300,500), (750,290), (1090,655), (1390,655), (1866,505), (1866,995), (1390,845)],
#              [(1866,505), (1866,995), (1390,845), (1090,845), (750,1210), (300,1000), (300,500), (750,290), (1090,655), (1390,655)]

numShips = len(paths)
x = 100#3800#2413
s = [(-i*x,0,0,0,0) for i in range(numShips)]
##s = [(0,0,0,0,0) for i in range(numShips)]
shipExternalStates = [state.ShipExternalState(*s[i]) for i in range(numShips)]

#
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
#   # POINTS FOR THE PARALLEL EDGES CONFIGURATION (AAAI 11 PAPER)
#   # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   numPointsInHalfPath = 18 # should be even
#   parallelEdgePoints = [ (500 + i * 250, 500) for i in range(numPointsInHalfPath) ] + list(reversed([ (500 + i * 250, 1500) for i in range(numPointsInHalfPath) ]))
#   patrolPoints = parallelEdgePoints
#
#
#   # paths for learning in the parallel edge example
#   tmp = [[patrolPoints[2*i], patrolPoints[2*i+1], patrolPoints[-(2*i+1) - 1], patrolPoints[-2*i - 1]] for i in range(numPointsInHalfPath / 2)]
#   paths = tmp + [list(reversed(path)) for path in tmp] + [patrolPoints, list(reversed(patrolPoints))]
#
#
#   # Ships initial state
#   numShips = len(paths)
#   x = 100#3800#2413
#   #s = [(-i*x,0,0,0,0) for i in range(numShips)]
#   s = [(0,0,0,0,0) for i in range(numShips)]
#   shipExternalStates = [state.ShipExternalState(*s[i]) for i in range(numShips)]
#
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
#   # POINTS FOR THE ISLAND EXAMPLE
#   # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   islandPoints =  [(640 ,990), (1150,830), (1310,1130), (1240,1580), (1190,1960), (1400, 1740), (1640, 1580), (1860,1510), (2140, 1420), (2640, 1260), (2640, 1620), (2860, 1680), (3140, 1700), (3440, 1980), (3940, 1820), (4260, 1480), (4480, 1120), (4640, 720), (4960, 300), (5240, 640), (5200, 940), (4880, 1320), (4850, 1700), (4480, 2020), (4000, 2240), (3340, 2500), (2860, 2500), (2720, 2260), (2580, 2120), (2080, 2320), (1840,2290), (1540, 2300), (1240, 2500), (750, 2640), (480, 2140), (660, 1660)]
#   patrolPoints = islandPoints    
#
#
#   l1 = patrolPoints
#   l2 = list(l1); l2.reverse()
#   l3 = [patrolPoints[35], patrolPoints[3], patrolPoints[4], patrolPoints[34]]
#   l4 = list(l3); l4.reverse()
#   l5 = [patrolPoints[4], patrolPoints[5], patrolPoints[31], patrolPoints[32]]
#   l6 = list(l5); l6.reverse()
#   l7 = [patrolPoints[6], patrolPoints[7], patrolPoints[29], patrolPoints[30]]
#   l8 = list(l7); l8.reverse()
#   l9 = [patrolPoints[10], patrolPoints[11], patrolPoints[27], patrolPoints[28]]
#   l10 = list(l9); l10.reverse()
#   l11 = [patrolPoints[13], patrolPoints[14], patrolPoints[24], patrolPoints[25]]
#   l12 = list(l11); l12.reverse()
#   l13 = [patrolPoints[14], patrolPoints[15], patrolPoints[16], patrolPoints[21], patrolPoints[22], patrolPoints[23], patrolPoints[24], patrolPoints[25]]
#   l14 = list(l13); l14.reverse()
#   paths = [l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11, l12, l13, l14]
#
#
#   ## Ships initial state
#   numShips = len(paths)
#   x = 100#3800#2413
#   s = [(-i*x,0,0,0,0) for i in range(numShips)]
#   ##s = [(0,0,0,0,0) for i in range(numShips)]
#   shipExternalStates = [state.ShipExternalState(*s[i]) for i in range(numShips)]
#
##~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~






##################### OTHER OPTIONS FOR START POSITIONS
#  # random start positions
#  ##############################################
#  shipExternalStates = [state.ShipExternalState(600, random.randint(0,400), orientation=random.random() * 360 - 180, speed=0, angularSpeed = 0) for i in range(len(ships))]
  
  
#  # Fixed position, for distance measurements
#  ##############################################
#  shipExternalStates = [state.ShipExternalState(100, 300, orientation=random.random() * 360 - 180, speed=0, angularSpeed = 0) for i in range(len(ships))]
  


