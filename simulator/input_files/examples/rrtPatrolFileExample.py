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



###################################################################
# This file defines three variables loaded into the simulator, 
# desribing the:
# 1. patrol-points (in the variable patrolPoints), which is a list of (x,y)
# points. For example: 
# patrolPoints = [(x0,y0), (x1,y1), ...]
#
# 2. patrol-paths (in the variable paths), which is a list of lists of (x,y)
# points. For example:
# p = patrolPoints
# paths = [ [p[0], p[5], p[3]], [p[3], p[7], p[11]], ... ]
#
# 3. initial states of the ships (in the variable shipExternalStates). To
# initialize, change only the s variable, which is a list of 5-tuples
# describing the ships initial states. A 5-tuple is (x, y, orientation, speed,
# angular speed). For example:
# s = [ [(100, 200, 0, 0, 0), (800, 400, 0, 0, 0), ... ]
#
#######################################################################

###################################################################
# FOR RRT WE CURRENTLY ASSUME THERE ARE TWO POINTS IN EACH PATH
###################################################################
patrolPoints = [(0, 0), (900, 100)]

paths = [[patrolPoints[0], patrolPoints[1]]]

numShips = len(paths)
#s = [(-i*x,0,0,0,0) for i in range(numShips)]
s = [(0,0,0,0,0) for i in range(numShips)]
shipExternalStates = [state.ShipExternalState(*s[i]) for i in range(numShips)]


