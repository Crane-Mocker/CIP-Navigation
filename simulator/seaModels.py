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



import numpy
from numpy import *
import random

class Sea:
  """
  This is a base class for the sea model.
  Different sea models can implement it, 
  and together with the ships states, they compose the world state.
  """

  # TODO: add a temporal model of changes to wind, waterCurrents, waves, and obstacles map
  def __init__(self, wind, waterCurrents, waves, obstacles):
    """
    Initialize the sea model.

    @type  wind: Wind
    @param wind: the Wind model

    @type  waterCurrents: WaterCurrents
    @param waterCurrents: the water currents model

    @type  waves: Waves
    @param waves: the waves model

    @type  obstacles: Obstacles
    @param obstacles: the sea obstacles model
    """

    #print 'Creating sea model'
    self.wind = wind
    self.waterCurrents = waterCurrents
    self.waves = waves
    self.obstacles = obstacles

  def __str__(self):
    res = "\nSea:\n" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res




###################################################
# This part defines the wind base class           #
# and some concrete class examples                #
###################################################
class Wind:
  """
  This is a base class for wind models.
  A wind is a mapping from location to a (2D) direction vector.
  """

  def __init__(self):
    print 'Creating wind model'

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the wind vector in it"
    abstract



class StaticConstantWind(Wind):
  """
  This class implements a wind that is constant in every point, 
  and never changes.
  """

  def __init__(self, directionX, directionY):
    "parameters define the constant wind direction vector"
    self.directionX = directionX
    self.directionY = directionY

  def __str__(self):
    res = "\nStaticConstantWind:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the wind vector in it"
    return numpy.array( [self.directionX, self.directionY] )




###################################################
# This part defines the water-currents base class #
# and some concrete class examples                #
###################################################
class WaterCurrents:
  """
  This is a base class for water-currents models.
  A water-current is a mapping from location to a (2D) direction vector.
  """

  def __init__(self):
    print 'Creating water-current model'

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the water-current vector in it"
    abstract

class StaticConstantWaterCurrents(WaterCurrents):
  """
  This class implements a water-current that is constant in every point,
  and never changes.
  """

  def __init__(self, directionX, directionY):
    "parameters define the constant water-current direction vector"
    self.directionX = directionX
    self.directionY = directionY

  def __str__(self):
    res = "\nStaticConstantWaterCurrents:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the wind vector in it"
    return numpy.array( [self.directionX, self.directionY] )


class StripeWaterCurrents(WaterCurrents):
  """
  This class implements a water-current that is constant and never changes,
  except one vertical stripe (with top and bottom part)
  that has different conditions.
  """

  def __init__(self, waterDirectionX, waterDirectionY, 
               stripeXStart, stripeXEnd, 
               stripeYmiddle,
               stripeTopWaterDirectionX, stripeTopWaterDirectionY,
               stripeBottomWaterDirectionX, stripeBottomWaterDirectionY):
    "parameters define the constant water-current direction vector"
    self.waterDirectionX = waterDirectionX
    self.waterDirectionY = waterDirectionY
    self.stripeXStart = stripeXStart
    self.stripeXEnd = stripeXEnd
    self.stripeYmiddle = stripeYmiddle
    self.stripeTopWaterDirectionX = stripeTopWaterDirectionX
    self.stripeTopWaterDirectionY = stripeTopWaterDirectionY
    self.stripeBottomWaterDirectionX = stripeBottomWaterDirectionX
    self.stripeBottomWaterDirectionY = stripeBottomWaterDirectionY


  def __str__(self):
    res = "\nStripeWaterCurrents:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the wind vector in it"
    if(x < self.stripeXStart or x > self.stripeXEnd):
      # outside the stripe
      return numpy.array( [self.waterDirectionX, self.waterDirectionY] )
    elif( y > self.stripeYmiddle ):
      return numpy.array( [self.stripeTopWaterDirectionX, self.stripeTopWaterDirectionY] )
    else:
      return numpy.array( [self.stripeBottomWaterDirectionX, self.stripeBottomWaterDirectionY] )
      
class IslandWaterCurrents(WaterCurrents):
  """
  Hard coded values For the Island example.
  We have a few islands, and two areas of strong currents
  between them.
  """
  def __init__(self, points):
    self.points = points

    b = self.points

    # First area of rough weather
    b5 = numpy.array(b[5])
    b34 = numpy.array(b[34])
    b3 = numpy.array(b[3])

    tmp = b34 - b5
    self.bay1Line = tmp * 1.0 / linalg.norm(tmp)

    # normalize and compute the normal for this line and intercept
    self.normalBay1Line = numpy.array([self.bay1Line[1], -self.bay1Line[0]])

    # Computed border values for bay1
    self.border1VerticalValueTop = dot(self.normalBay1Line, b5) 
    self.border1VerticalValueBottom = dot(self.normalBay1Line, b3)
    self.border1VerticalValueMiddle = (self.border1VerticalValueTop +
                                       self.border1VerticalValueBottom) * 1.0 / 2
    self.border1HorizonalValueTop = dot(self.bay1Line, b34) + 50
    self.border1HorizonalValueBottom = dot(self.bay1Line, b5) - 50
    self.border1HorizonalValueMiddle = (self.border1HorizonalValueTop +
                                        self.border1HorizonalValueBottom) * 1.0 / 2
    

    # Second area of rough water
    b25 = numpy.array(b[25])
    b24 = numpy.array(b[24])
    b13 = numpy.array(b[13])
    b14 = numpy.array(b[14])

    tmp = b25 - b24
    self.bay2Line = tmp * 1.0 / linalg.norm(tmp)

    # normalize and compute the normal for this line and intercept
    self.normalBay2Line = numpy.array([self.bay2Line[1], -self.bay2Line[0]])

    # Computed "top-bottom" border values for bay1
    self.border2VerticalValueTop = dot(self.normalBay2Line, b24) + 50
    self.border2VerticalValueBottom = dot(self.normalBay2Line, b13) - 50
    self.border2VerticalValueMiddle = (self.border2VerticalValueTop + 
                                       self.border2VerticalValueBottom) * 1.0 / 2
    self.border2HorizonalValueTop = dot(self.bay2Line, b25)
    self.border2HorizonalValueBottom = dot(self.bay2Line, b14)
    self.border2HorizonalValueMiddle = (self.border2HorizonalValueTop + 
                                        self.border2HorizonalValueBottom) * 1.0 / 2

  def __str__(self):
    res = "\nIslandWaterCurrents:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

  def getSpeedVectorInLocation(self, x, y):
    # If in bay 1 - hard to cross
    p = numpy.array((x,y))
    dot1 = dot(p, self.normalBay1Line)
    dot2 = dot(p, self.bay1Line)
    inSideBay1 = ((self.border1VerticalValueBottom <= dot1 <= self.border1VerticalValueTop) and
                  (self.border1HorizonalValueBottom <= dot2 <= self.border1HorizonalValueTop))
    if inSideBay1:
      return self.bay1Line * 8 
    
    dot1 = dot(p, self.normalBay2Line)
    dot2 = dot(p, self.bay2Line)
    inSideBay2 = (self.border2VerticalValueBottom <= dot1 <= self.border2VerticalValueTop and
                  self.border2HorizonalValueBottom <= dot2 <= self.border2HorizonalValueTop)
    if inSideBay2:
      return self.normalBay2Line * -8 

    # other wise, random mild current
    waterSpeed = random.weibullvariate(1,2) 
    waterDirection = random.randint(0,360)
    return numpy.array( waterSpeed * cos(radians(waterDirection)), waterSpeed * sin(radians(waterDirection)) )

class VerticalWaterCurrents(WaterCurrents):
  """
  Defines north->south or south->north currents.
  Was used for the AAAI 11 experiments.
  """

  def __init__(self, regionsAndCurrentsList):
    """
    @type  regionsAndCurrentsList: list of 3-tuples
    @param regionsAndCurrentsList: a list of 3-tuples, each defines a stripe: (minX, maxX, currentValue), where currentValue is signed to account for direction.
    """
    self.regionsAndCurrentsList = regionsAndCurrentsList

  def getSpeedVectorInLocation(self, x, y):
    """
    Check stripe and return corresponding current.
    """
    for p1, p2, current in self.regionsAndCurrentsList:
      x1 = p1[0]
      x2 = p2[0]
      if x1 <= x <= x2:
        # vertical normal multiplied by the current value
        return current * numpy.array([0,1])
    return numpy.array([0, 0])
      
      
  def __str__(self):
    res = "\nVerticalWaterCurrents:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

    
    




###################################################
# This part defines the waves base class          #
# and some concrete class examples                #
# We don't yet have a good waves model so we just #
# ignore it.                                      #
###################################################
class Waves:
  """
  This is a base class for wave models.
  Currently, we assume that waves just have a direction vector 
  in each location. We ignore the wave height, and the fact 
  that is has some period, and averaging everything out,
  to assume a direction vector in each (x,y) point.
  """

  def __init__(self):
    print 'Creating water-current model'

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the avg speed vector, for this location"
    abstract

class StaticConstantWaves(Waves):
  """
  This class implements waves that are constant in every point,
  and never change.
  """

  def __init__(self, directionX, directionY):
    "Parameters define the constant wave speed vector"
    self.directionX = directionX
    self.directionY = directionY

  def __str__(self):
    res = "\nStaticConstantWaves:" 
    for member, value in vars(self).items():
      res += " " + member + "=" +  str(value)
    res += "\n"
    return res

  def getSpeedVectorInLocation(self, x, y):
    "Given a location, returns the wave vector in it"
    return numpy.array( [self.directionX, self.directionY] )




###################################################
# This part defines Obstacles in the sea          #
###################################################
class Obstacle:
  """
  An obstacle in the sea. Currently an obstacle is just a 
  straight line. This class support general polygon but,
  for now we only want line composed of 2 points. It is
  possible to construct complex obstacles just from 
  straight lines.
  
  Therfore, for now IGNORE the following:
  [Currently implemented 
  as a list of points, where connecting each two 
  consecutive points by line defines the obstacle's
  shape. The first point should be equal to the last one.]
  """
  def __init__(self, points):
    """
    @type  points: a list of (x,y) tuples
    @param points: a list of points - connecting each consecutive pair of points by line defines the obstacle's shape.
    """
    self.points = points
#    if points[0] != points[-1]: 
#      raise Exception("First point of an obstacle should be equal to the last point")
    if len(points) != 2:
      raise Exception("We currently support only straight line obstacles (though this could easily be changed)")

  def __str__(self):
    res = "\nObstacle:" 
#    for member, value in vars(self).items():
#      res += " " + member + "=" +  str(value)
    for p in self.points:
      res += " " + str(point)
    res += "\n"
    return res

  def getBorder(self):
    """
    @rtype:  tuple
    @return: a list of points defining the shape of the obstacle
    """
    return self.points



class Obstacles:
  """
  This class models the obstacles part of the sea.
  """
  def __init__(self, obstacleList):
    """
    Initialize the set of obstacles.

    @type  obstacleList: a list of Obstacle.
    @param obstacleList: a list of obstacles. 
    """
    print 'Creating obstacles model'
    self.obstacles = obstacleList

  def getObstaclesList(self):
    return self.obstacles

  def __str__(self):
    res = "\nObstacles:" 
#    for member, value in vars(self).items():
#      res += " " + member + "=" +  str(value)
    for o in self.obstacles:
      res += str(o)
    res += "\n"
    return res

