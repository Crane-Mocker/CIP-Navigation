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

###############################################################
# POINTS FOR THE PARALLEL EDGES CONFIGURATION (AAAI 11 PAPER) #
###############################################################
task = 'TASK_STATIC_PATROL'
# override sea model files as needed
waterFile = 'waterCurrents_parallel_edge_example.py'

# all possible edges - for agent exploration
# patrol points
numPointsInHalfPath = 8 # should be even
parallelEdgePoints = [ (500 + i * 250, 500) for i in range(numPointsInHalfPath) ] + list(reversed([ (500 + i * 250, 1000) for i in range(numPointsInHalfPath) ]))
patrolPoints = parallelEdgePoints
# paths 
tmp = [[patrolPoints[2*i], patrolPoints[2*i+1], patrolPoints[-(2*i+1) - 1], patrolPoints[-2*i - 1]] for i in range(numPointsInHalfPath / 2)]
paths = tmp + [list(reversed(path)) for path in tmp] + [patrolPoints, list(reversed(patrolPoints))]


# Ships initial state
numShips = len(paths)
shipInitialStates = [state.ShipExternalState(0,0,0,0,0) for i in range(numShips)]

# we ignore rules of the sea in this case
rulesOfTheSea = False

# This means data collection - learning distances between patrol points
resultsType = 'edgeGraphData'
