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




class PrimitiveMessage:
  """
  A primitive message that is send from an agent to
  other agent / other agents / the simulation environment.
  """

  # message types
  TYPE_VISIT         = 'MSG_TYPE_VISIT'
  TYPE_EDGE_LENGTH   = 'MSG_TYPE_EDGE_LENGTH'
  TYPE_START_PATROL  = 'MSG_TYPE_START_PATROL'
  TYPE_POSITION      = 'MSG_TYPE_POSITION'
  TYPE_PATROL_DIVISION = 'MSG_TYPE_PATROL_DIVISION'
  TYPE_REACHED_START = 'MSG_TYPE_REACHED_START'
  TYPE_TRACKING_POINT = 'MSG_TYPE_TRACKING_POINT'
  TYPE_NAVIGATION_CENTRAL_POINT = 'MSG_TYPE_NAVIGATION_CENTRAL_POINT'
  TYPE_POSITIONS_ASSIGNMENT = 'MSG_TYPE_POSITIONS_ASSIGNMENT'

  # receiver options
  TO_SIMULATOR = -1
  TO_ALL       = -2
  # SUBSET_OF_AGENTS # TODO - need to add support for that

  def __init__(self, to, type, content):
    self.to = to
    self.type = type
    self.content = content

  def __str__(self):
    res = "PrimitiveMessage:"
    res += " to=" + str(self.to)
    res += " type=" + str(self.type)
    res += " content=" + str(self.content)
    res += "\n"
    return res


class CompositeMessage:
  """
  Class for general Message that is created by an agent.
  A CompositeMessage is just a (possibly empty) list of PrimitiveMessage.
  """
  def __init__(self):
    self.primitiveMsgs = []
  
  def __str__(self):
    res = "\nCompositeMessage:\n"
    for a in self.primitiveMsgs:
      res += str(a)
    res += "\n"
    return res
    

  def appendMsg(self, msg):
    """
    Adding a primitive message to the list of outgoing messages.

    @type  msg: PrimitiveMessage
    @param msg: the message to be added to the list
    """
    self.primitiveMsgs.append(msg)
