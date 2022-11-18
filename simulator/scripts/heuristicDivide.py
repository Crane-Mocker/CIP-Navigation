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



###################################################
# This file implements the algorithms from the 
# ship patrol paper. All the algorithms assume 
# an outerplannar graph
# 
# This file executes as an independent module - 
# take a look at the __main__ function at the bottom
#
# AN EXAMPLE OF HOW TO RUN A FULL FLOW:
# 1. go to the src directory
# 2. run: python main.py -q (a file edgeGraphData.py is created
# 3. run: python scripts/heuristicDivide.py -n <num ships> (a file heuristicDivideOutput.py is created and copied to the input_files directory
# 4. run: python main.py -r visit_times -f heuristicDivideOutput.py (a file visit_times.res is created with the worst case frequency 
###################################################


import sys
import os
scriptdir = os.path.abspath(os.path.dirname(sys.argv[0]))
sys.path.append(os.path.join(scriptdir, '..'))

from geometry import *
import state

import copy
from math import *
import numpy 
import shutil
sys.path.append('/usr/lib/graphviz/python/')
sys.path.append('/usr/lib64/graphviz/python/')
#import gv

# Import pygraph
from pygraph.classes.graph import graph
from pygraph.classes.digraph import digraph
from pygraph.algorithms.searching import breadth_first_search
from pygraph.algorithms.accessibility import *
#from pygraph.readwrite.dot import write


# Used to turn-off prints
class NullDevice():
  def write(self, s):
      pass






# draws a graph
#def draw(gr):
#  dot = write(gr)
#  gvv = gv.readstring(dot)
#  gv.layout(gvv,'dot')
#  gv.render(gvv,'png','a.png')



## Graph creation
#gr = graph()
#
## Add nodes and edges
#gr.add_nodes(['1', '2', '3', '4', '5', '6', '7', '8', '9'])
#gr.add_edge(('1','2'), 1)
#gr.add_edge(('2','3'), 1)
#gr.add_edge(('3','1'), 1)
#gr.add_edge(('3','4'), 10)
#gr.add_edge(('4','5'), 1)
#gr.add_edge(('5','6'), 1)
#gr.add_edge(('6','4'), 1)
#gr.add_edge(('6','1'), 10)



### Initial graph I used
#gr.add_edge(('1','2'))
#gr.add_edge(('2','3'))
#gr.add_edge(('3','1'))
#gr.add_edge(('3','4'))
#gr.add_edge(('4','5'))
#gr.add_edge(('5','6'))
#gr.add_edge(('6','4'))
#gr.add_edge(('6','7'))
#gr.add_edge(('7','8'))
#gr.add_edge(('8','9'))
#

################################
# Algorithm for finding cycles
################################
def getComponents(componentsStructure):
  """
  Create a mapping of [component number]->[its (sorted) nodes list]
  from a given components-structure.

  @type  componentsStructure: dictionary
  @param componentsStructure: pairing that associates each node to its connected component. 

  @rtype:  dictionary of lists
  @return: a mapping from component number to is (sorted) nodes list.
  """
  components = {}
  for val in componentsStructure.values():
    components[val] = []

  for key, val in componentsStructure.items():
    components[val].append(key)

  # sort the nodes in each component
  for val in components.values():
    val.sort()

  return components


def findSingletons(componentsStructure):
  """
  Find singleton components in a structure of bi-connected components.

  @type  componentsStructure: dictionary
  @param componentsStructure: pairing that associates each node to its connected component. 
  
  @rtype:  list 
  @return: a list of nodes that are singleton components.
  """
  components = getComponents(componentsStructure)

  singletons = []
  for nodes in components.values():
    if len(nodes) == 1:
      node = nodes [0]
      singletons.append(node)
  
  return singletons
  

def addConnectedComponent(component, componentsStructure):
  """
  Adds a connected component to the biconnectedComponents.

  @type  component: list
  @param component: nodes that are part of the same component

  @type  componentsStructure: dictionary
  @param componentsStructure: pairing that associates each node to its connected component. 

  @rtype:  None 
  @return: no return value - change is inplace
  """
  componentIds = componentsStructure.values()
  maxId = max(componentIds)
  newId = maxId + 1
  for node in component:
    componentsStructure[node] = newId


def cycles(biconnectedComponentsStructure):
  """
  Given a bi-connected-components structure, and assuming outerplannar graph,
  we create a list of the maximal cycles of each of the components, 
  assuming that a sorting of a component's nodes is the cycle.
  
  @type  biconnectedComponents: dictionary
  @param biconnectedComponents: pairing that associates each node to its connected component. 

  @rtype:  list of lists 
  @return: A list of cycles (a cycle is just like a pygraph cycle: a list of nodes)
  """
  components = getComponents(biconnectedComponentsStructure)

  for i in components.keys():
    # assume that nodes sorting gives a cycle, according to 
    # the way we defined it
    components[i].sort()
    
  cycles = components.values()

#  for k, val in cycles.items():
#    print 'cycle[', k, ']= ', val
  return cycles



def findDisjointMaximalCycles(graph):
  """
  Find Disjoint, Maximal, non-overlapping Cycles in the (outerplanar) graph.
  If no such cycles, return None.
  
  @type  graph: pygraph.classes.graph
  @param graph: the graph in which we find cycles

  @rtype:  list of lists, or None 
  @return: A list of disjoint cycles in the graph or None if couldn't find
           (a cycle is just like a pygraph cycle: a list of nodes)
  """

  # TODO: copy? or allow to change?
  gr = copy.deepcopy(graph)

  # remove bridges from graph
  bridges = cut_edges(gr)
  for edge in bridges:
    gr.del_edge(edge)


  # verify no articulation points and get biconnnected components
  articulationPoints = cut_nodes(gr)
  if len(articulationPoints) > 0:
    # we need to skip this graph - we want disjoint cycles
    return None
  else:
    # no articulation points - connected is biconnected (?)
    biconnectedComponents = connected_components(gr)

    # add bridges that are biconnectedComponents back to the graph.
    singletons = findSingletons(biconnectedComponents)
    for bridge in bridges:
      if bridge[0] in singletons and bridge[1] in singletons:
        # add it to biconnected
        # (cannot add back the bridge because might be run over by 
        #  a later addition of a bridge containing on of its nodes:
        #  o--o--o--o
        # Also, for long chains it might not add all possible bridges
        addConnectedComponent(bridge, biconnectedComponents)

    return cycles(biconnectedComponents)

    
#################################################################
# assignKAgents
#################################################################
def cycleLength(cycle, dgraph):
  """
  Computes the cycleLength of a cycle in a graph.

  @type  cycle: list
  @param cycle: the list of nodes in the cycle

  @type  dgraph: digraph
  @param dgraph: the full graph from which the cycles are taken

  @rtype:  float
  @return: the cycleLength (length) of a cycle in a graph
  """
  cycleLen = 0
  numNodes = len(cycle)

  # If 1 node, no need to compute
  if numNodes > 1:
    for i in range(numNodes):
      node = cycle[i]
      nextNode = cycle[(i + 1) % numNodes]
      edge = (node, nextNode)
      if not dgraph.has_edge(edge):
        raise Exception("Cannot compute cycle length. Edge " + str(edge) + " does not exist in cycle " + str(cycle))
      cycleLen += dgraph.edge_weight(edge)
#      print >> sys.stderr, 'edge', edge, 'cycleLen', cycleLen

  return float(cycleLen)

def distributeAdditionalAgents(agentsToDistribute, dcycles, frequencies, K):
  """
  In assignKAgents, in case we achieved the frequency requirement and 
  we have more available agents left, we distribute them one at a time,
  to a cycle with the worst frequency.

  @type  availableAgents: int
  @param availableAgents: number of additional ships available to be assigned to cycles

  @type  dcycles: a list of lists
  @param dcycles: a list of directed cycles that is sent by ref to be appended to

  @type  frequencies: a list of numbers
  @param frequencies: a list of the frequencies corresponding to each cycle assignment, sent by ref, to be appended to

  @rtype  K: a list of numbers
  @return K: a list of the number of agents per cycle, corresponding to the dcycles list
  """
  availableAgents = agentsToDistribute 
  # if there are available agents - divide each one to the worst-idleness cycle
  while availableAgents > 0:
    # Begin sanity check
    if max(frequencies) == 0:
      raise "I am having more robots that nodes in some subdivision - what should I do?"
    # End sanity check

    worstCycleIndex = frequencies.index(max(frequencies))
    oldK = K[worstCycleIndex]
    K[worstCycleIndex] += 1
    newK = K[worstCycleIndex]
    # edge case: if we have more ships than nodes, put a ship per node as a guard
    if newK >= len(dcycles[worstCycleIndex]):
      # keep a copy of the cycle nodes
      cycleNodes = list(dcycles[worstCycleIndex])
      # remove all data that is related to this cycle
      del dcycles[worstCycleIndex]
      del frequencies[worstCycleIndex]
      del K[worstCycleIndex]
      # add singleton cycles instead
      for node in cycleNodes:
        dcycles.append([node])
        frequencies.append(0)
        K.append(1)
    else:
      frequencies[worstCycleIndex] *= float(oldK) / newK
    availableAgents -= 1


def assignKAgents(graph, ucycles, f, k):
  """
  Tries to assign the k agents to m cycles so that max idleness 
  is less than f. In a cycle, the agents can either:
  1. Cycle, or 
  2. Each agent guards a single node.
  In addition, the function finds the shortest direction (CW or CCW) for each cycle,
  and returns resulting directed cycles, cycle-frequencies, and robot assignments

  @type  graph: graph
  @param graph: the full graph from which the cycles are taken

  @type  ucycles: list of lists
  @param ucycles: undirected cycles in a graph to travel

  @type  f: float
  @param f: maximal allowed node idleness

  @type  k: int
  @param k: number of robots

  @rtype:  3-tuple of lists
  @return: three lists: a list of directed cycles (after finding the shortest direction), 
                        a list of cycle-frequencies, and a list of #robots (per cycle)
  """
  #TODO: change parameter to be digraph (different weights for pairs
  #      of edges)
  availableAgents = k
  maxIdleness = 0
  # agent assignments
  dcycles = []
  frequencies = []
  K = [] 

  # For each cycle we find:
  # 1. How many agents it needs
  # 2. What would be the frequency
  # 3. What is the cycle direction the agents should travel
  print 'CYCLES', ucycles
  for c in ucycles:
    numNodes = len(c)
    if numNodes == 1:
      # special case: singleton
      neededAgents = 1
      actual_f = 0 
      cycle = c # doesn't matter, no cycling
    else:
      # find direction of shorter cycle
      c1 = list(c)
      c2 = list(c); c2.reverse()
      length1 = cycleLength(c1, graph)
      print 'c1', c1, 'length1', length1
      length2 = cycleLength(c2, graph)
      print 'c2', c2, 'length2', length2
      minLength = float(min(length1, length2))

      # Use as much agents as needed, if still left enough.
      neededAgents = int(ceil(minLength / f) + 0.1) # safe round?
      # Special case
      if neededAgents >= numNodes:
        neededAgents = numNodes
        actual_f = 0 # guarding a node
        cycle = c    # doesn't matter, no cycling
      else:
        actual_f = minLength / neededAgents
        cycle = c1 if length1 <= length2 else c2



    if neededAgents > availableAgents:
      # not enough robots for assignment
      return None

    if neededAgents == numNodes:
      # special case: break cycles into "guarding" agents
      for node in cycle:
        dcycles.append([node])
        frequencies.append(0)
        K.append(1)
    else:
      # Save the number of agents and the current idleness 
      dcycles.append(cycle)
      frequencies.append(actual_f)
      K.append(neededAgents)

    availableAgents -= neededAgents

  if availableAgents > 0:
    distributeAdditionalAgents(availableAgents, dcycles, frequencies, K)

  print "(dcycles, frequencies, K) = ", (dcycles, frequencies, K)
  return (dcycles, frequencies, K)



#################################################################
# DivideTo2Circles
#################################################################
def divideTo2Circles(digraph, f, k):
  """
  Returns the best division of the graph into two components,
  with maximal idleness of at most f, using k robots.
  If no such division exists, return None.

  @type  digraph: digraph
  @param digraph: an outerplanar directed graph to be divided. If there is an edge 
                  between pair of nodes, there must also be an edge in the opposite direction.

  @type  f: float
  @param f: maximal allowed node idleness

  @type  k: int
  @param k: number of robots

  @rtype:  a tuple of lists, or None
  @return: a tuple of: <list of cycles, list of cycle-frequencies, list of #robots assigned per cycle>
  """
  print 'divideTo2Circles()'
  # create an undirected copy of the digraph
  ugraph = graph()
  ugraph.add_nodes(digraph.nodes())
  for e in digraph.edges():
    if e[0] < e[1]:
      ugraph.add_edge(e)

  # pygraph has 2 directed edges for each undirected edge
  Divisions = []
  edges = [e for e in ugraph.edges() if e[0] < e[1]]
  for i in range(len(edges)):
    for j in range(i + 1, len(edges)):
      gr = copy.deepcopy(ugraph)
      gr.del_edge(edges[i])
      gr.del_edge(edges[j])
      print 'removing', edges[i], edges[j]

      # find UNDIRECTED cycles
      ucycles = findDisjointMaximalCycles(gr)
      if ucycles == None:
        print 'findDisjointMaximalCycles failed, continuing'
        continue

      result = assignKAgents(digraph, ucycles, f, k)
      if result == None:
        print 'assignKAgents failed, continuing'
        continue

      C, F, K = result
      Divisions.append( (C, F, K) )

  if Divisions == []:
    return None
  # we minimize the max cost
  best = min(Divisions, key=lambda x: max(x[1]))
  print 'divideTo2Circles() RETURNS:', best
  return best



####################################################
# heuristicDivide
####################################################
def inducedGraph(gr, nodes):
  """
  Builds an induced DIRECTED graph given a set of nodes.

  @type  gr: graph
  @param gr: a directed graph

  @type  nodes: list
  @param nodes: a list of nodes on which we induce a graph

  @rtype:  a graph
  @return: a graph that is induced by the given nodes
  """
  if not gr.DIRECTED:
    raise Exception('I can only handle directed graphs')
  g = digraph()
  g.add_nodes(nodes)
  for e in gr.edges():
    if e[0] in nodes and e[1] in nodes:
      g.add_edge(e, gr.edge_weight(e))
  return g 

# TODO: add @rtype in all functions

def heuristicDivide(graph, f, k):
  """
  Heuristically find the best circles division.
  It does that by calling divideTo2Circles() and 
  then calling itself recursively 
  on the graphs resulting from divideTo2Circles().
  We assume the function is initially called with f = cycle-length / k
  such that at least the full cycle is a valid division, and
  from there the function tries to improve.

  @type  graph: graph
  @param graph: an outerplanar graph to be divided.

  @type  f: float
  @param f: maximal allowed node idleness

  @type  k: int
  @param k: number of robots

  @rtype:  a list of tuple-pairs
  @return: a list of pairs of <cycle, freq, #robots>
  """
  print 'heuristicDivide()'
  Division = []
  result = divideTo2Circles(graph, f, k)

  # base case: either couldn't divide, or found that 1 cycle is the best
  if result == None or len(result[0]) == 1:
    # return the current cycle and same k, accounting for cycle direction
    nodes1 = list(graph.nodes()); nodes1.sort()
    length1 = cycleLength(nodes1, graph)

    nodes2 = list(nodes1); nodes2.reverse()
    length2 = cycleLength(nodes2, graph)

    if length1 <= length2:
      # prefer sorted in case of tie-break (no special reason, just so we could unit-test it)
      cycle = nodes1
      length = length1
    else:
      cycle = nodes2
      length = length2
    #return [(cycle, k)]## DANIEL RESTORE and delete next 2 lines
    f = length / k
    return [(cycle, f,  k)]

  # tail recursion - division worked
  cycles, F, K = result
  for c, f, k in zip(cycles, F, K):
    if f == 0:
      # A robot is assigned per node - no need to further divide
      # Division.append( (c, k) ) ## DANIEL RESTORE and delete next line
      Division.append( (c, f, k) )
    else:
      g = inducedGraph(graph, c)
      res = heuristicDivide(g, f, k)
      Division += res

  return Division
  
#TODO: check for directionality: assignKAgents might need to return the cycle direction as well?


#############################################################
# Functions that compose the flow that uses heuristicDivide
#############################################################
def buildGraphFromEdgeLengths(edgeLengths, points):
  """
  Translate measured edge lengths into nodes and edges in a directed graph.

  @type  edgeLengths: a dictionary that maps an edge (p1, p2) to a list of numbers
  @param edgeLengths: maps an edge (p1, p2) to a list measured lengths (travel times) of this edge.

  @type  points: a list of 2-tuples [(x1,y1),...,(x_n,y_n)]
  @param points: a list of all points of interest

  @rtype:  a digraph
  @return: a directed graph built from the input parameters
  """
#  print 'Number of edges', len(edgeLengths)
  gr = digraph()
  #gr.add_nodes(['points' + str(i) for i in range(len(points))])
  gr.add_nodes([i for i in range(len(points))])
  for i in range(len(points)):
    for j in range(len(points)):  
      edge = (points[i],points[j])
      if edge in edgeLengths:
        #gr.add_edge(('points'+str(i), 'points'+str(j)), mean(edgeLengths[edge]))
        gr.add_edge((i, j), numpy.mean(edgeLengths[edge]))
  return gr


def initShipsPositionsAndPaths(heuristicDivideResult, points, gr):
  """
  Given the ships division computed by heuristicDivide(), compute the ships
  initial positions (and do a cyclic permutation on their paths, to adjust 
  for their location in the circle)

  @type  heuristicDivideResult: a list of tuple-pairs
  @param heuristicDivideResult: a list of pairs of <cycle, freq, #robots>

  @type  points: a list of 2-tuples [(x1,y1),...,(x_n,y_n)]
  @param points: a list of all points of interest

  @type  gr: digraph
  @param gr: the directed graph of the problem

  @rtype:  a pair of ([list of ShipExternalStates], [list of lists])
  @return: a pair of ([list of the ships' external states], [list of ships' paths])
  """
  shipExternalStates = [] # initial ship states
  paths = [] # will hold the final agent paths
  # for each cycle, we assign numAgents
  for pathIndices, ignore, numAgents in heuristicDivideResult:
    path = [points[i] for i in pathIndices]
#    print 'pathIndices', pathIndices, 'path', path

    # compute ship initial states, and initial configuration
    length = cycleLength(pathIndices, gr)
    timeDiffBetweenAgents = length / numAgents

    # assign first agent at the begining
    if len(path) > 1: 
      orient = orientation2D(path[0], path[1]) 
    else:
      orient = 0 
    p = path[0]
    shipExternalStates.append(state.ShipExternalState(p[0], p[1], orient, 0, 0)) 
#    print 'assign agent 1'
    paths.append(path)
#    print 'len(paths)', len(paths)
#    print 'SHIP NUM:', len(paths)

    # assign all other agents
    agentsLeft = numAgents - 1
#    print 'agentsLeft', agentsLeft
    cumDist = 0
    numNodes = len(pathIndices)
    for i in range(1, numNodes) + [0]:  
      # cumulative distance is computed while scanning the cycle
      currentEdgeWeight = gr.edge_weight((pathIndices[(i-1)%numNodes], pathIndices[i])) 
#      print 'currentEdgeWeight', currentEdgeWeight
      cumDist += currentEdgeWeight
#      print 'cumDist', cumDist
#      print 'i', i, 'len(pathIndices)', len(pathIndices)
      while agentsLeft > 0 and cumDist > timeDiffBetweenAgents: # 'while' instead of 'if' because might be very long edge
        # agent should be assigned somewhere on the current edge
        residual = cumDist - timeDiffBetweenAgents
#        print 'residual', residual
#        print 'assign agent ', agentsLeft
        currentPoint = numpy.array(path[i])
#        print 'currentPoint: point', i, ':', currentPoint
        prevPoint = numpy.array(path[(i-1)%numNodes])
#        print 'prevPoint: point', i, ':', prevPoint
        edgeFraction = float(currentEdgeWeight - residual) / currentEdgeWeight 
        p = prevPoint + (currentPoint - prevPoint) * edgeFraction 
#        print 'POINT:', p
        orient = orientation2D(p, currentPoint)
#        print 'orientation2D', orient
        shipExternalStates.append( state.ShipExternalState(p[0], p[1], orient, 0, 0) )

        # assign the path, make a cyclic offset according to the agent
        agentPath = path[i:] + path[:i]
#        print 'agentPath', agentPath
        paths.append(agentPath)
#        print 'len(paths)', len(paths)

        # reset
        cumDist = residual
        agentsLeft -= 1
  return shipExternalStates, paths

def default(str):
  return str + ' [Default: %default]'

def readCommand( argv ):
  """
  Processes the command used to run the simulation from the command line.
  """
  from optparse import OptionParser
  usageStr = """
  USAGE:      python heuristicDivide.py <options>
  EXAMPLE:    python heuristicDivide.py --numShips 5
  """
  parser = OptionParser(usageStr)
  
  parser.add_option('-n', '--numShips', dest='numShips', type='int',
                    help=default("Number of ships available for patrol"), default='1')
  parser.add_option('-i', '--inputFile', dest='inputFile', 
                    help=default('input file that init the patrol points and the edge lengths'), 
                    default='edgeGraphData.py')
  parser.add_option('-o', '--outputFile', dest='outputFile', 
                    help=default('output file that should init patrolPoints, paths, and shipExternalStates'), 
                    default='heuristicDivideOutput.py')
  parser.add_option('-d', '--debug', action='store_true', dest='debug',
                    help=default('turns on debug prints'),
                    default=False)
  
  options, other = parser.parse_args(argv)
  if len(other) != 0: 
    raise Exception('Command line input not understood: ' + other)
  args = dict()

  args['numShips'] = options.numShips
  args['inputFile'] = options.inputFile
  args['outputFile'] = options.outputFile
  args['debug'] = options.debug

  return args


def runWithCmdLineArgs(numShips, inputFile, outputFile, debug):
  """
  Run the heuristicDivide algorithm with file input/output.
  """
  # in: edgeLengths, patrolPoints
  exec(compile(open(inputFile).read(), inputFile, 'exec'))
  if 'patrolPoints' not in locals() or 'edgeLengths' not in locals(): 
    raise Exception('File ' + file + ' must initialize patrolPoints and edgeLengths variables')
  patrolPoints, paths, shipExternalStates = run(numShips, patrolPoints, edgeLengths, debug)
  outfile = open(outputFile, 'w')
  outfile.write('patrolPoints=' + str(patrolPoints) + '\n\n')
  outfile.write('paths=' + str(paths) + '\n\n')
  # writing ship states in two lines
  outfile.write('shipInitialPositions=' + str([(s.x, s.y, s.orientation, s.speed, s.angularSpeed) for s in shipExternalStates]) + '\n')
  #outfile.write('shipExternalStates= [state.ShipExternalState(*a[i]) for i in range(len(paths))]\n')
  outfile.close()
  # NOTE: copy outfile to be used by main.py
  shutil.copy(outputFile, 'input_files')


def run(numShips, patrolPoints, edgeLengths, debug):
  """
  Run the heuristicDivide algorithm with in-memory input/output
  """

#  origstdout = sys.stdout
#  origstderr = sys.stderr
#  if not debug:
#    sys.stdout = NullDevice()
#    sys.stderr = NullDevice()

#  edgeLengths = env.state.edgeLengths # shorthand
  gr = buildGraphFromEdgeLengths(edgeLengths, patrolPoints)
  heuristicDivideResult = heuristicDivide(gr, 9999999, numShips)
  print 'heuristicDivideResult', heuristicDivideResult
  shipExternalStates, paths = initShipsPositionsAndPaths(heuristicDivideResult, patrolPoints, gr)

#  sys.stdout = origstdout
#  sys.stderr = origstderr

  return patrolPoints, paths, shipExternalStates
    


  # out: patrolPoints, paths, shipExternalStates


if __name__ == '__main__':
  """
  Runs heuristicDivide() flow, as an independent module
  The main function called when running the simulation

  > python main.py

  See the usage string for more details.

  > python main.py --help
  """
  args = readCommand( sys.argv[1:] ) # Get command line flags
  print 'Running simulation with the following options:'
  for k, v in sorted(args.items()):
    print k, '=', v
  runWithCmdLineArgs( **args )

