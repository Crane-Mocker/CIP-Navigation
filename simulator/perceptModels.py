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



#
#class PerceptTypes:
#  # TODO: values are ints for efficiency, change to other, safer types?
#  COMPLETE_STATE = 0
#
#  # NOTE: update the list to reflect the above members
#  LEGAL_TYPES = [PerceptTypes.COMPLETE_STATE]
#
#
#class Percept:
#  """
#  A general percept that a ship can generate, 
#  and an agent can receive.
#
#
#  arguments:
#  type - a percept type that should be in PerceptTypes
#  data - a mapping between fields and values, specific to each percept type
#  """
#  def __init__(self, type, data):
#    if type not in PerceptTypes.LEGAL_TYPES:
#      raise UnknownPerceptError(type)
#
#    self.type = type
#    self.data = data

##################################################
#
# This file defines different percept classes.
# Each ship can use a subset of them.
#
#
#
#
#
#
##################################################

class PerceptList:
  """
  A general percept that a ship can generate, 
  and an agent can receive.
  It is a container that holds a list of different 
  percept objects.
  """
  def __init__(self):
    self.percepts = []

  def appendPercept(self, percept):
    self.percepts.append(percept)

  def getPercepts(self):
    return self.percepts



class CompleteStatePercept:
  """
  A percept that senses the full, complete, state.
  This is an instance of a specific percept that is appended
  inside the class of PerceptList, however it doesn't yet
  inherit from any base "Percept" class because the interface
  of such class is not finalized yet. Once more percepts are 
  introduced, this class might be revised.
  """
  def __init__(self, state):
    self.state = state
