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


#############################################
#
# This file is a config file for water currents. 
# The result of executing this file 
# is an assignment of an object of
# type WaterCurrents to the variable 
# "waterCurrents". You can define similar
# files, and set them in the task definition 
# language file, to be used in your task
#
#############################################



import seaModels

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# 
# PARALLEL EDGES CASE FOR AAAI
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
numPointsInHalfPath = 18 # should be even
parallelEdgePoints = [ (500 + i * 250, 500) for i in range(numPointsInHalfPath) ] + list(reversed([ (500 + i * 250, 1500) for i in range(numPointsInHalfPath) ]))


# Vertical water currents for the parallel edge example

#currentDirection = lambda x : 30 if x % 4 == 1 else -30
#currentDirection = lambda x : 22 if x % 4 == 1 else -22
currentDirection = lambda x : 15 if x % 4 == 1 else -15
#currentDirection = lambda x : 0 if x % 4 == 1 else 0

# from parallelEdgeConfiguration.py
waterCurrents = seaModels.VerticalWaterCurrents( [(parallelEdgePoints[i], parallelEdgePoints[i+1], currentDirection(i)) for i in range(1, numPointsInHalfPath - 1, 2)] )
# 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
