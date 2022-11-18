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
# This file contains implementation of trees for the RRT algorithm.
# Trees are only responsible for storing nodes, finding a 
# closest node, and finding a path from a root to a node.
###################################################################

import random

from actions import *


class RRTNode:
  """
  A node in the RRT tree.
  Contains the state, a pointer to the parent, 
  and the edge from the parent.
  The edge from the parent is the action that takes it
  from the parent's state to the currrent state.
  """
  def __init__(self, shipState, parent, edge=CompositeAction()):
    self.state = shipState
    self.parent = parent
    self.edge = edge





class RRTTree:
  """
  Interface for an RRT.
  Based on the paper: http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.36.7457
  """

  def addNode(self, node):
    "Adds a new node to the tree."
    abstract

  def getPathTo(self, currentState):
    """
    Returns a path (a sequence of actions) from root to currentState.
    
    Keyword arguments:
    currentState -- ShipExternalState to which we want a path from the 
                    initial state.

    Returns:
    A path of actions that would bring us to currentState

    """
    abstract
  
  def findClosestNode(self, newState):
    """
    Find closest node in the RRT w.r.t. x, y, orientation. 

    Keyword arguments:
    newState -- ShipExternalState that is going to be part of the
                new tree node.

    Returns:
    RRTNode which is the noded closest to 'newState' in the current RRT

    """
    abstract



class RRTTreeXYOrientationEuclidDist(RRTTree):
  """
  Implements a simple RRT, which assumes a state is
  (x,y,orientation), and the distance between two states
  is euclidean, after normalizing their attributes to
  a similar scale.

  """
  def __init__(self, initialShipState, distanceUnit, angleUnit):
    """Initialize the tree.

    Keyword arguments:
    initialShipState -- initial ShipExternalState that would be 
                        the root of the tree

    distanceUnit -- 
    angleUnit -- Both are used for normalization of x, y and angle
                 to the same scale, to determine similarity (or distance)
                 between two points (x1, y1, ang1) and (x2, y2, ang2).
                 These could be, for instance, the amount of distance / orientation
                 a ship can travel in a given time unit.
    """
    self.root = RRTNode(initialShipState, parent=None)
    self.nodes = [self.root]

    self.distanceUnit = distanceUnit 
    self.angleUnit = angleUnit


  def addNode(self, node):
    self.nodes.append(node)

  def getPathTo(self, currentState):
    """Returns a path (a sequence of actions) from root to currentState."""
    actions = []

    # Scan for the node of currentState and return the path to it
    for node in self.nodes:
      if node.state == currentState: # by-reference comparison
        # Found a node, construct a path of actions that lead to it
        currentNode = node
        while currentNode != None: # None is the parent of self.root
          actions.append(currentNode.edge)
          currentNode = currentNode.parent
        actions.reverse()
        
        # Append a stopping action
        action = CompositeAction()
        action.appendAction(PrimitiveAction("stop", []))
        actions.append(action)

        return actions

    # If got here didn't find a node
    raise Exception("Didn't find the node in the tree")
  
  def getNodesPathTo(self, currentState):
    """Returns a path OF NODES from root to currentState."""
    nodes = []

    # Scan for the node of currentState and return the path to it
    for node in self.nodes:
      if node.state == currentState: # by-reference comparison
        # Found a node, construct a path of actions that lead to it
        currentNode = node
        while currentNode != None: # None is the parent of self.root
          nodes.append(currentNode)
          currentNode = currentNode.parent
        nodes.reverse()

        return nodes

    # If got here didn't find a node
    raise Exception("Didn't find the node in the tree")

  def findClosestNode(self, newState):
    """
    Find closest node in the RRT w.r.t. x, y, orientation. 
    Distance is euclidean, where x, y and orientation are
    normalized.

    Keyword arguments:
    newState -- ShipExternalState that is going to be part of the
                new tree node.

    Returns:
    RRTNode which is the noded closest to 'newState' in the current RRT

    """
    # Scanning for the minimal node in the pythonic way (no for loops)
    states = [node.state for node in self.nodes]
    xDiffs = [abs(newState.x - nodeState.x) / self.distanceUnit for nodeState in states]
    yDiffs = [abs(newState.y - nodeState.y) / self.distanceUnit for nodeState in states]
    angDiffs = [abs(newState.orientation - nodeState.orientation) / self.angleUnit for nodeState in states]
    distances = map( lambda x,y,ang : x * x + y * y + ang * ang, xDiffs, yDiffs, angDiffs)

    minDistance = min(distances)
    epsilon = 0.00000001
    minDistIndexes = [i for i, dist in enumerate(distances) if abs(dist - minDistance) < epsilon]
    chosenIndex = random.choice(minDistIndexes)
    return self.nodes[chosenIndex]
 
