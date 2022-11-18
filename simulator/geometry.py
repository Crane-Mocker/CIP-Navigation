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
import numpy as np

def distance2D(a, b):
  "2D distance between two points (x1, y1) and (x2, y2)"
  xdiff = b[0] - a[0]
  ydiff = b[1] - a[1]
  diagSquared = xdiff * xdiff + ydiff * ydiff
  return sqrt(diagSquared) 

def orientation2D(a, b):
  """
  Orientation of the vector b-a.
  The results is between -180 and 180 degrees
  """
  # TODO: replace by a vector class?
  xdiff = b[0] - a[0]
  ydiff = b[1] - a[1]
  return atan2(ydiff, xdiff) / pi * 180

def computeShortestTurn(fromAng, toAng):
  """
  Computes that shortest angle (positive or negative) 
  between 2 angles.
  """
  # TODO: replace by a vector class?
  turn = toAng - fromAng
  candidate1 = turn
  candidate2 = turn - 360
  candidate3 = turn + 360
  absTurn2OrigTurn = { abs(candidate1) : candidate1, 
                       abs(candidate2) : candidate2,
                       abs(candidate3) : candidate3 }
  return absTurn2OrigTurn[ min([abs(candidate1), abs(candidate2), abs(candidate3)]) ]

def averagePoint2D(pointList):
  """
  return the average point in a list of (x,y) points
  """
  totalX = 0.0
  totalY = 0.0
  for p in pointList:
    totalX += p[0]
    totalY += p[1]
  avgX = totalX / len(pointList)
  avgY = totalY / len(pointList)
  return np.array([avgX, avgY])
