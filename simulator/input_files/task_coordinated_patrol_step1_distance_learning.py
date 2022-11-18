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


task = 'TASK_STATIC_PATROL'
# override sea model files as needed
waterFile = 'water_currents_basic_example.py'

# all possible edges - for agent exploration
# patrol points
patrolPoints = [(300,500), (750,290), (1090,655), (1390,655), (1866,505), (1866,995), (1390,845), (1090,845), (750,1210), (300,1000)]
# paths
path1 = patrolPoints
path2 = list(patrolPoints); path2.reverse()
path3 = [patrolPoints[2], patrolPoints[3], patrolPoints[6], patrolPoints[7]]
path4 = [patrolPoints[7], patrolPoints[6], patrolPoints[3], patrolPoints[2]]
paths = [path1, path2, path3, path4]

# define initial positions for ships - number of ships must equal 
# the number of paths
shipInitialStates = [ 
  # 4 ships that start patrolling
  state.ShipExternalState(x=0, y=0, orientation=0, speed=0, angularSpeed=0),
  state.ShipExternalState(x=-100, y=0, orientation=0, speed=0, angularSpeed=0),
  state.ShipExternalState(x=-200, y=0, orientation=0, speed=0, angularSpeed=0),
  state.ShipExternalState(x=-300, y=0, orientation=0, speed=0, angularSpeed=0)
]

# we ignore rules of the sea in this case
rulesOfTheSea = False

# This means data collection - learning distances between patrol points
resultsType = 'edgeGraphData'

