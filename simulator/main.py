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




import sys
import os
import random
import operator
from math import *
from numpy import *
import time

import seaModels
import shipModels
import state
import agents
import simulationEnvironment
import factories

#from heuristicDivide import *
from geometry import *

def default(str):
  return str + ' [Default: %default]'

def validateOptionCombinations(args):
  """
  This functions validates we didn't mistakenly chosen options
  that don't work with each other.
  If options are verified we just return to the the program
  execution. If the options are incorrect, we gracefully exit.
  """
#  if( args['shipType'] == 'basic' 
#      and args['worldModel'] == 'complete' 
#      and args['strategy'] == 'circling' ):
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'holonomicrrt' ):
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'pid' ):
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'measuredist' ):
#     print 'Warning: change this combination in the future'
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'staticpatrol' ):
#     print 'Warning: change this combination in the future'
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'coordinatedpatrol' ):
#     print 'Warning: change this combination in the future'
#     return

#  if( args['shipType'] == 'basic'
#      and args['worldModel'] == 'complete'
#      and args['strategy'] == 'rrt' ):
#     print 'Warning: change this combination in the future'
#     return

  # currently disabled option checking - it doesn't seem to be needed
  return

  print 'Illegal options combination - exiting...'
  sys.exit()

  # TODO: Once we support mixed ship types, check combinations 
  #       per each ship

# Used to turn-off prints
class NullDevice():
  def write(self, s):
      pass

def readCommand( argv ):
  """
  Processes the command used to run the simulation from the command line.
  """
  from optparse import OptionParser
  usageStr = """
  USAGE:      python main.py <options>
  """
  parser = OptionParser(usageStr)
  
  parser.add_option('-q', '--quietTextGraphics', action='store_true', dest='quietGraphics', 
                    help=default('Generate minimal output and no graphics'), default=False )    
  parser.add_option('-n', '--numSteps', dest='numSteps', type='int',
                    help=default("Number of steps to simulate"), default=10000)
  parser.add_option('-s', '--speed', dest='speed', type='int',
                    help=default("frames per second in GUI mode (each frame is one simulated second)"), default=60)
  parser.add_option('-i', '--inputFilesDir', dest='inputFilesDir', 
                    help=default('the directory in which input files are searched for'), 
                    metavar='DIR', default='input_files')
  parser.add_option('-f', '--taskFile', dest='taskFile', 
                    help=default('A task-definition-language file. The file is searched for under the input files directory (specified by the flag -i)'), 
                    default='task_multiple_ships_tracking_multiple_targets.py')
  parser.add_option('-r', '--randomSeed', dest='randomSeed', type='int',
                    help=default('reloads a random seed - for simulation replay'), default=None)
  parser.add_option('-d', '--debug', action='store_true', dest='debug',
                    help=default('turns on debug prints'),
                    default=False)

#  parser.add_option('-z', '--zoom', type='float', dest='zoom', 
#                    help=default('Zoom the size of the graphics window'), default=1.0)
#  parser.add_option('-t', '--frameTime', dest='frameTime', type='float',
#                    help=default('Time to delay between frames; <0 means keyboard'), default=0.1)
  
  options, other = parser.parse_args()
  if len(other) != 0: 
    raise Exception('Command line input not understood: ' + other)
  args = dict()

  # if options.randomSeed is not None, this fixes a random seed.
  # if it is None, then system time is used
  random.seed(options.randomSeed)
  
  # Choose GUI / non-GUI
  if options.quietGraphics:
    args['withDisplay'] = False
  else:
    args['withDisplay'] = True
    

  args['numSteps'] = options.numSteps
  args['speed'] = options.speed
  args['inputFilesDir'] = options.inputFilesDir
  args['taskFile'] = options.taskFile
  args['debug'] = options.debug

  validateOptionCombinations(args)
  
  return args


######################################################
# Initialization Functions -                         #
# Those function are here temporarily                # 
# until there is a flexible solution                 #
# for loading different models, perhaps              #
# as a part of the task definition language          #
######################################################
def initWindFromFile(filePath):
  # just execute the (python) file and return a variable generated
  exec(compile(open(filePath).read(), filePath, 'exec'))
  return wind

def initWaterCurrentsFromFile(filePath):
  # just execute the (python) file and return a variable generated
  exec(compile(open(filePath).read(), filePath, 'exec'))
  return waterCurrents

def initWavesFromFile(filePath):
  # just execute the (python) file and return a variable generated
  exec(compile(open(filePath).read(), filePath, 'exec'))
  return waves

def initObstaclesFromFile(filePath):
  # just execute the (python) file and return a variable generated
  exec(compile(open(filePath).read(), filePath, 'exec'))
  return obstacles


######################################################
# Results writing functions                          #
######################################################
def writeResults(resultsType, env, patrolPoints):
  """
  Write the simulation results.

  TODO: this function needs to be further organized
  once we have a better picture regarding what
  types of results are usually needed.
  Also the function arguments should be revised as a part
  of the function reorganization.

  @type  resultsType: string
  @param resultsType: a string identifier for the type of result to write

  @type  resultsType: SimulationEnvironment
  @param resultsType: The simulation environment, used to extract data from

  @type  patrolPoints: array of (x,y) tuples
  @param patrolPoints: All the points that participate in the patrol. This is required when writing data for heuristicDivide algorithm, and might be removed at some point.
  """
  #TODO: this function needs to be further organized
  #      once we have a better picture regarding what
  #      types of results are usually needed.
  if resultsType == 'visit_times':

    ts = time.time()
    print >> sys.stderr, "timestamp of res: ", ts
    f = open(resultsType + '_' + str(ts) + '.res', 'w')

    # write only if all points have at least two visits
    if len(env.visitTimes.values()) >= 1 and min([len(l) for l in env.visitTimes.values()]) >= 2:
      # Simulation results, K+1, K-1
      point2freq = dict([(point, mean([l[i] - l[i-1] for i in range(1,len(l))])) for point, l in env.visitTimes.items() ])
      if len(point2freq) > 0:
        worstCaseFreq1 = max(point2freq.values())
        f.write('WORSTCASE FREQ: ' + str(worstCaseFreq1) + '\n')
        f.write('patrolPoints=' + str(patrolPoints) + '\n')
        f.write('visitTimes=' + str(env.visitTimes) + '\n')

    f.close()

  elif resultsType == 'edgeGraphData':

    ts = time.time()
    print >> sys.stderr, "timestamp of res: ", ts
    f = open(resultsType + '_' + str(ts) + '.py', 'w')
    f.write('patrolPoints=' + str(patrolPoints) + '\n')
    f.write('edgeLengths=' + str(env.edgeLengths) + '\n')
    f.close()


###############
# Main Function
###############
#def run(withDisplay, shipType, worldModel, strategy, numShips, inputFilesDir, taskFile):
# disabled numShips, because determined by the number of paths
#def run(withDisplay, shipType, worldModel, strategy, inputFilesDir, taskFile, numSteps, rulesOfTheSea):
def run(withDisplay, inputFilesDir, taskFile, numSteps, speed, debug):
  """
  The main flow of the application. The arguments are generated by command line options, or their defaults.

  @type  withDisplay: boolean
  @param withDisplay: Tells whether to run in a GUI mode or in non-GUI mode

  @type  inputFilesDir: string
  @param inputFilesDir: The directory in which the input files are 

  @type  taskFile: string
  @param taskFile: A task-definition-language file 

  @type  numSteps: int
  @param numSteps: Number of steps to simulate

  @type  speed: int
  @param speed: frames-per-second (each frame is one simulated second)
  """
 
  origstdout = sys.stdout
  origstderr = sys.stderr
  if not debug:
    sys.stdout = NullDevice()


  ########### TASK INITIALIZATION - DEFINED BY TASK DEFINITION FILE ###########


  # defaults
  windFile = 'wind.py'
  waterFile = 'waterCurrents.py'
  wavesFile = 'waves.py'
  obstaclesFile = 'obstacles.py'
  shipType = 'basic'
  strategy = 'staticpatrol'
  worldModel = 'complete'
  rulesOfTheSea = True #False
  resultsType = 'edgeGraphData'



  # LOAD TASK DEFINITION FILE, POSSIBLY OVERRIDES DEFAULTS

  pathToTaskFile = os.path.join(inputFilesDir, taskFile)
  exec(compile(open(pathToTaskFile).read(), pathToTaskFile, 'exec'))



  # CREATE A TASK TO BE EXECUTED

  # task variable should be loaded from task definition file, along with other related vars
  try:
    task
  except Exception as e:
    print >> sys.stderr, '-E- Exception Raised:', e
    sys.exit(1)
  if task == 'TASK_STATIC_PATROL':
    try:
      # checking existence
      shipInitialStates
      patrolPoints
      paths
    except Exception as e:
      print >> sys.stderr, '-E- Exception Raised:', e
      sys.exit(1)
    #shipInitialStates = [state.ShipExternalState(*shipInitialPositions[i]) for i in range(len(paths))]
    strategy = 'staticpatrol' #overriding strategy

  elif task == 'TASK_PATROL_WITH_ONLINE_ASSIGNMENT':
    try:
      # checking existence
      patrolPoints
      edgeLengths
      shipInitialStates
      roles
      if len(shipInitialStates) != len(roles):
        raise Exception("shipInitialStates and roles must be at the same length")
    except Exception as e:
      print >> sys.stderr, '-E- Exception Raised:', e
      sys.exit(1)

    strategy = 'coordinatedpatrol' #overriding strategy

  elif task == 'TASK_TARGET_TRACKING':
    try:
      # checking existence
      roles
      shipInitialStates
      patrolPoints
      paths
      trackingDistance
      positionOffsets
      if len(shipInitialStates) != len(roles):
        raise Exception("shipInitialStates and roles must be at the same length")
    except Exception as e:
      print >> sys.stderr, '-E- Exception Raised:', e
      sys.exit(1)

    strategy = 'targettracking' #overriding strategy 

  else: 
    raise Exception('Unknown task type')

  try:
    # checking existence
    shipInitialStates
    strategy
  except Exception as e:
    print >> sys.stderr, '-E- Exception Raised:', e
    sys.exit(1)




  ############# AGENT INITIALIZATION ##############

  numShips = len(shipInitialStates)

  # CREATE SHIPS

  if shipType == 'basic':
    shipFactory = factories.BasicShipFactory()
  else:
    raise Exception('Unknown ship type: ' + shipType)
  ships = [shipFactory.create() for i in range(numShips)]
  # TODO: When we support mixed ship models,
  #       allocate an agent based on ship, and send it the possible actions 
  #       for this ship



  # CREATE SHIP-CONTROLLING-AGENTS

  # Choose a world model factory
  if worldModel == 'complete':
    worldModelFactory = factories.CompleteWorldModelFactory()
  else:
    raise Exception('Unknown world model type: ' + worldModel)

  # Choose a decision making strategy
  if strategy == 'circling':
    strategyFactories = [factories.CirclingStrategyFactory() for i in range(len(ships))]
  elif strategy == 'rrt':
    strategyFactories = [factories.RRTStrategyFactory(paths, rulesOfTheSea) for i in range(len(ships))]
  elif strategy == 'pid':
    strategyFactories = [factories.PIDStrategyFactory(rulesOfTheSea) for i in range(len(ships))]
  elif strategy == 'measuredist':
    strategyFactories = [factories.MeasureEdgeLengthsFactory() for i in range(len(ships))]
  elif strategy == 'staticpatrol':
    strategyFactories = [factories.AgentStaticPatrolStrategyFactory(paths, rulesOfTheSea) for i in range(len(ships))]
  elif strategy == 'coordinatedpatrol':
    # regular ships + joining ships
    strategyFactories = []
    for role in roles:
      if role == 'originalPatroller':
        strategyFactories.append( 
            factories.AgentCoordinatedPatrolStrategyFactory(patrolPoints, 
               edgeLengths, rulesOfTheSea)) 
      elif role == 'joinsLater':
        strategyFactories.append(
            factories.AgentJoiningPatrolStrategyFactory(patrolPoints, 
                edgeLengths, rulesOfTheSea))
      else:
        raise Exception('Unknown ship role: ' + role)
  elif strategy == 'targettracking':
    trackedShipIndices = [i for i in range(len(roles)) if roles[i] == 'tracked']
    trackingShipIndices = [i for i in range(len(roles)) if roles[i] == 'tracker']
    if len(trackingShipIndices) != len(positionOffsets):
      raise Exception('Number of tracking ships must equal to the length of positionOffsets')
    if len(trackedShipIndices) != len(paths):
      raise Exception('Number of tracked ships must equal to the number of paths')
    # regular ships + joining ships
    strategyFactories = []
    for role in roles:
      if role == 'tracked':
        strategyFactories.append( 
            factories.AgentStaticPatrolStrategyFactory(paths, rulesOfTheSea))
      elif role == 'tracker':
        strategyFactories.append(
            factories.AgentTrackingPatrolStrategyFactory(trackedShipIndices, 
                trackingShipIndices, trackingDistance, positionOffsets, rulesOfTheSea))
      else:
        raise Exception('Unknown ship role: ' + role)
  else:
    raise Exception('Unknown decision making strategy: ' + strategy)

  # Allocate agents with the above factories
  shipAgents = [agents.Agent( worldModel = worldModelFactory.create(shipIndex=i),
                              strategy = strategyFactories[i].create(agentIndex=i) ) 
                              for i,s in enumerate(ships)]
  print 'shipAgents', len(shipAgents)



  # LOAD ENVIRONMENT MODEL

  # task definition file possible overrided default filenames
  wind = initWindFromFile(os.path.join(inputFilesDir, windFile))
  waterCurrents = initWaterCurrentsFromFile(os.path.join(inputFilesDir, waterFile))
  waves = initWavesFromFile(os.path.join(inputFilesDir, wavesFile))
  obstacles = initObstaclesFromFile(os.path.join(inputFilesDir, obstaclesFile))
  sea = seaModels.Sea(wind, waterCurrents, waves, obstacles)


  ######################### SIMULATION ENVIRONMENT ############################


  # world state
  initialState = state.State( sea, ships, shipInitialStates )

  # Simulation module
  env = simulationEnvironment.SimulationEnvironment( initialState, shipAgents )

  # start the simulation
  import mainGUI
  if withDisplay:
    mainGUI.runGui(env, shipType, worldModel, strategy, numSteps, speed)
  else:
    env.run(numSteps)

  # write results
  writeResults(resultsType, env, patrolPoints)

  sys.stdout = origstdout




if __name__ == '__main__':
  """
  The main function called when running the simulation

  > python main.py

  See the usage string for more details.

  > python main.py --help
  """
  args = readCommand( sys.argv[1:] ) # Get game components based on input
  print 'Running simulation with the following options:'
  for k, v in sorted(args.items()):
    print k, '=', v
  run( **args )
