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



##########################################################################
# This code was moved as a part of the cleaning of main.py
# Here there is just a function that could create these results if needed.
# 
# In order to run the function, we need to send it three parameters:
# heuristicDivideResult - the result that heuristicDivide returns
# visitTimes - the recorded visit times when simulating 
#              a patrol created using heuristicDivide
# patrolPoints - the patrolPoints used in main.py
##########################################################################
from numpy import *

# Input params for the function can be found as follows:
# 1. when running main.py for a result of worstcase frequency, the resulting 
#    file includes the patrolPoints and visitTimes
# 2. heuristicDivideResult should be printed when running heuristicDivide.py
def computeKPlus1KMinus1(heuristicDivideResult, visitTimes, patrolPoints):

  point2freq = dict([(point, mean([l[i] - l[i-1] for i in range(1,len(l))])) for point, l in visitTimes.items() ])
  worstCaseFreq1 = max(point2freq.values())

  
  f = open('KPlus1KMinus1.res','w')
  # COMPUTE K+1, K-1
  # create a list of cycle frequencies
  cycleFrequencies = []
  K = []
  cycleLengths = []
  for pathIndices, ignore, numAgents in heuristicDivideResult:
    cycleFrequencies.append( max([point2freq[patrolPoints[i]] for i in pathIndices]) )
    K.append(numAgents)
    cycleLengths.append(len(pathIndices))

#    print cycleFrequencies
#    print K
#    print cycleLengths

  # K+1
  # for each cycle, try to add 1 and see whether the frequency improves over the best one
  bestKplus1Freq = worstCaseFreq1
  for i in range(len(cycleFrequencies)):
    tmp = list(cycleFrequencies)
    if K[i] + 1 == cycleLengths[i]:
      # new-#agents == #nodes  ==> frequency is 0
      tmp[i] = 0
    else:
      tmp[i] *= K[i] / float(K[i]+1)
#      print 'tmp[i]', i, tmp[i]
    if max(tmp) < bestKplus1Freq:
      bestKplus1Freq = max(tmp)
#    print  >> sys.stderr, 'K+1', bestKplus1Freq  
  f.write('K+1: ' + str(bestKplus1Freq) + '\n')

  # K-1
  # for each cycle, try to subtract 1 and see whether the frequency improves over the best one
  bestKminus1Freq = float('inf')
  for i in range(len(cycleFrequencies)):
    if K[i] > 1:
      tmp = list(cycleFrequencies)
      tmp[i] *= K[i] / float(K[i]-1)
      if max(tmp) < bestKminus1Freq:
        bestKminus1Freq = max(tmp)
#    print  >> sys.stderr, 'K-1', bestKminus1Freq
  f.write('K-1: ' + str(bestKminus1Freq) + '\n')
  f.close()

