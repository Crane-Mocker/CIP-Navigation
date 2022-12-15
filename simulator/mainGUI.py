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



#!/usr/bin/env python
# -*- coding: utf-8 -*-
# generated by wxGlade 0.6.3 on Thu Jul 22 09:48:20 2010

import Tkinter
import tkMessageBox
import wx
import os
import pygame
import random
import time
import copy
from math import *
from navigationTactics import AgentRRTTactic
from agents import AgentStaticPatrolStrategy
# begin wxGlade: extracode
# end wxGlade


# Color constants
RED = (255,0,0)
YELLOW = (255, 255, 0)
DARKGREEN = (0, 100, 0)
DODGERBLUE = (30, 144, 255)
PINK = (255, 192, 203)
GRAY60 = (153, 153, 153)
DARKORANGE = (255, 140, 0)
BLUEVIOLET = (138, 43, 226)
ORANGE = (255, 165, 0)
AQUAMARINE = (127, 255, 212)
SADDLEBROWN = (139, 69, 19)
CHOCOLATE2 = (238, 118, 33)
GREEN3 = (0, 205, 0)
DEEPPINK = (255, 20, 147)
HOTPINK = (255, 105, 180)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
COLORS = [RED, DARKGREEN, DODGERBLUE, PINK, GRAY60, DARKORANGE, BLUEVIOLET, ORANGE, AQUAMARINE, SADDLEBROWN, CHOCOLATE2, GREEN3, DEEPPINK, YELLOW, HOTPINK]

# TODO: after finishing working with wx-glade, change all functions here to 
# start with a lower case

class MainGUIWindow(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MainGUIWindow.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.notebook_1 = wx.Notebook(self, -1, style=0)
        
        
        self._pygameDataInitialized = False 
        self._framecounter = 0 
        self.lastScreenX = 0
        self.lastScreenY = 0

        self.stepNum = 0

        # Zoom Level
        self.zoom = 3.375 #5.0 #1.0
        self.lastZoom = self.zoom

        # should be initialized in initData()
        self.simEnv = None
        self.updateBackgroundHooks = [] # a list of hooks
        self.numStepsToSimulate = 0
        self.agent2RRTData = {}

        # set a color scheme
        self.darkBackground = False
        if self.darkBackground:
          self.bgColor = (0,0,0)
        else:
          #self.bgColor = (255,255,255)
          self.bgColor = (135,206,250)


        # Menu Bar
        self.frame_1_menubar = wx.MenuBar()
        self.file = wx.Menu()
        self.file.Append(wx.NewId(), "&Open", "", wx.ITEM_NORMAL)
        self.file.Append(wx.NewId(), "&Save", "", wx.ITEM_NORMAL)
        self.file.AppendSeparator()
        self.quit = wx.MenuItem(self.file, wx.NewId(), "&Quit\tCtrl+Q", "", wx.ITEM_NORMAL)
        self.file.AppendItem(self.quit)
        self.frame_1_menubar.Append(self.file, "&File")
        wxglade_tmp_menu = wx.Menu()
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Edit")
        wxglade_tmp_menu = wx.Menu()
        self.frame_1_menubar.Append(wxglade_tmp_menu, "&Help")
        self.SetMenuBar(self.frame_1_menubar)


        # Status Bar 
        self.frame_1_statusbar = self.CreateStatusBar(1, 0)

        
        # Tool Bar
        self.frame_1_toolbar = wx.ToolBar(self, -1)
        self.SetToolBar(self.frame_1_toolbar)
        runId = wx.NewId()
        self.frame_1_toolbar.AddLabelTool(runId, "Run", wx.Bitmap(os.path.join(os.path.dirname( __file__ ), "images", "player_play.png"), wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Run", "Starts the simulation")
        zoominId = wx.NewId()
        self.frame_1_toolbar.AddLabelTool(zoominId, "Zoom In", wx.Bitmap(os.path.join(os.path.dirname( __file__ ), "images", "zoom-in.png"), wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Zoom In", "Zoom In")
        zoomoutId = wx.NewId()
        self.frame_1_toolbar.AddLabelTool(zoomoutId, "Zoom Out", wx.Bitmap(os.path.join(os.path.dirname( __file__ ), "images", "zoom-out.png"), wx.BITMAP_TYPE_ANY), wx.NullBitmap, wx.ITEM_NORMAL, "Zoom Out", "Zoom out")



        # Notebook
        self.notebook_1_pane_1 = wx.Panel(self.notebook_1, -1)
        self.notebook_1_pane_2 = wx.Panel(self.notebook_1, -1)
        self.notebook_1_pane_3 = wx.Panel(self.notebook_1, -1)


        self.__set_properties()
        self.__do_layout()


        # Bindings
        self.Bind(wx.EVT_TOOL, self.OnRunClick, id=runId)        
        self.Bind(wx.EVT_TOOL, self.OnZoominClick, id=zoominId)        
        self.Bind(wx.EVT_TOOL, self.OnZoomoutClick, id=zoomoutId)        
        self.Bind(wx.EVT_MENU, self.OnQuit, self.quit)
        # end wxGlade

        

    def __set_properties(self):
        # begin wxGlade: MainGUIWindow.__set_properties
        self.SetTitle("CIP Battleship Simulator")
        self.SetSize((1300, 800))
#        self.SetToolTipString("TODO: tooltip")
        self.frame_1_statusbar.SetFieldsCount(2)
        self.frame_1_statusbar.SetStatusWidths([-3,-2])
        # statusbar fields
        frame_1_statusbar_fields = ["frame_1_statusbar", ""]
        for i in range(len(frame_1_statusbar_fields)):
            self.frame_1_statusbar.SetStatusText(frame_1_statusbar_fields[i], i)
        self.frame_1_toolbar.Realize()
        # end wxGlade



    def __do_layout(self):
        # begin wxGlade: MainGUIWindow.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        #self.notebook_1.AddPage(self.notebook_1_pane_1, "Params")
        self.notebook_1.AddPage(self.notebook_1_pane_2, "Simulation")
        #self.notebook_1.AddPage(self.notebook_1_pane_3, "Results")
        sizer_1.Add(self.notebook_1, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

    def initData(self, env, shipType, worldModel, strategy, numSteps, speed):
      """Init application specific data"""
      self.setSimEnv(env)
      self.setConfigurationOptions(shipType, worldModel, strategy)
      self.setNumStepsToSimulate(numSteps)
      self.fps = float(speed) #60.#30.# 60.#200.0 #30.0
      # This data is used for eficient deletion of old drawn trees
      for i in range(len(self.simEnv.shipAgents)):
        self.agent2RRTData[i] = None

    def setSimEnv(self, env):
      "The GUI queries the env for its state and display it"
      self.simEnv = env
      self.initialSimEnv = copy.deepcopy(env)


    def setConfigurationOptions(self, shipType, worldModel, strategy):
      """Passing the cmd-line configuration options.
      Based on what the user chose on the cmd-line, we might have
      different GUI configurations.
      """
      # Assigning callback functions, called in UpdateBackground()
      if strategy == 'holonomicrrt' or strategy == 'rrt':
        self.updateBackgroundHooks.append(self.drawRRTHook)
        self.updateBackgroundHooks.append(self.basicPatrolHook)
      elif strategy == 'pid' or strategy == 'measuredist':
        self.updateBackgroundHooks.append(self.drawStartEndHook)
      elif strategy == 'staticpatrol': 
        self.updateBackgroundHooks.append(self.basicPatrolHook)
        # uncomment for drawing the artificial obstacles
        self.updateBackgroundHooks.append(self.drawObstacleHook)
        self.updateBackgroundHooks.append(self.drawRRTHook)
      elif strategy == 'coordinatedpatrol':
        self.updateBackgroundHooks.append(self.basicPatrolHook)
        # uncomment for drawing the artificial obstacles
        self.updateBackgroundHooks.append(self.drawObstacleHook)
        self.updateBackgroundHooks.append(self.drawRRTHook)
        self.updateBackgroundHooks.append(self.onlineHeuristicDivideHook)
      elif strategy == 'targettracking': 
        self.updateBackgroundHooks.append(self.trackingHook)
        # uncomment for drawing the artificial obstacles
        self.updateBackgroundHooks.append(self.drawObstacleHook)
        self.updateBackgroundHooks.append(self.drawRRTHook)
      else:
        self.updateBackgroundHooks.append(self.noopHook)

    def setNumStepsToSimulate(self, numSteps):
      self.numStepsToSimulate = numSteps

    def OnRunClick(self, event):
        "Callback for the run button, starts the simulation"
	#tkMessageBox.showinfo('test', 'ok')

        # If simulation environment not initialized, do nothing
        if self.simEnv == None:
          print 'World Environment is not initialized, ignoring click'
          return

        # This code is called only once, for initialization
        if not self._pygameDataInitialized:
            # window handle
            hwnd = self.notebook_1_pane_2.GetHandle()
            os.environ['SDL_WINDOWID'] = str(hwnd)
            pygame.init()
            # display
            x,y = self.notebook_1_pane_2.GetSizeTuple()
            self._surface = pygame.display.set_mode((x,y))
            self._surface.fill(self.bgColor)
            self.lastScreenX, self.lastScreenY = x, y
            # ship images and positions
            #self.ship = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','ship.png'))#.convert()
            #self.ship = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','ship2Large.png'))#.convert()
            #self.ship = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','ship2.png'))#.convert()
            self.ship = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','battleship.png'))#.convert()
            # rectangles of the real, computed, ship center positions
            """
            self.shipPositions = [self.ship.get_rect().move(600, random.randint(0,400)) for i in range(10)] 
            """
            self.shipPositions = [self.ship.get_rect().move(self.screen(state.x), self.screen(state.y)) for state in self.simEnv.state.shipExternalStates] 
            # rectangles of rotated ships including rotation
            self.shipRotatedPositions = list(self.shipPositions)
            """
            self.angle = -90
            """

            # Objects for the backgroud if needed
            #self.obstacles = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','obstacles_whitebg.png'))#.convert()
            self.obstacles = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','obstacles_transparent_bg.png')).convert()
#            self.waypoint = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','targetLarge.png'))#.convert()
            self.waypoint = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','target-3.png'))#.convert()
            self.waypoint_dynamic = pygame.image.load(os.path.join(os.path.dirname( __file__ ), 'images','target_dynamic.png'))#.convert()


            # Timers
            self.StartTimers(event)

            self._pygameDataInitialized = True

    def OnZoominClick(self, event):
      "Zoom in callback."
      self.zoom /= 1.5

    def OnZoomoutClick(self, event):
      "Zoom out callback."
      self.zoom *= 1.5

    def StartTimers(self, event):
        "Initialize timers for different events"

        # The main timer: updating frames
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.Update, self.timer)
        self.timespacing = 1000.0 / self.fps
        self.timer.Start(self.timespacing, False)
        self.i = 0


        # Auxiliary timer: makes sure background is repainted from time to time
        self.backgroundTimer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.UpdateBackground, self.backgroundTimer)
        self.backgroundTimer.Start(500) # every 1/2 second
        self.UpdateBackground(None)


      
    def Update(self, event):
        "Call back for the timer event"
        if self.stepNum >= self.numStepsToSimulate:
          # quit simulation
          self.Close() # TODO: is it correct to call Close both from here and from OnQuit()?
        self.stepNum += 1
        self.UpdateScreen()


    def UpdateScreen(self): # TODO: None is just for enabling timer based updates that are not related to any data
        """
        Draws the next frame in the animation.

        Uses the "dirty rectangles" method.
        This method only updates changed parts of the screen,
        in contrast to updating all the screen each frame.
        It gives significant performance improvement.
        """

        self.simEnv.step()
        newShipStates = self.simEnv.state.shipExternalStates

        
        # delete all old rectangles
        t1 = time.time()
        for pos in self.shipRotatedPositions:
          self._surface.fill(self.bgColor, pos)
        t2 = time.time()
        #print "erase:", (t2-t1)*1000.0
        

        # draw all the new rectangles
        t1 = time.time()
        rotate = pygame.transform.rotate
        oldRotatedPositions = list(self.shipRotatedPositions)
        self.shipRotatedPositions = []
        for i,shipState in enumerate(newShipStates):
          # update the real positions
          """
          speed = i
          transformAngle = (self.angle * -1) - 90 # from ^ image coord to "top-left based" coords
          xTranslation = round( speed * cos(radians(transformAngle)) )
          yTranslation = round( speed * sin(radians(transformAngle)) )
          newpos = pos.move(xTranslation, yTranslation)
          self.shipPositions[i] = newpos
          """
          oldX, oldY = self.shipPositions[i].center
          xDiff = self.screen(shipState.x) - oldX
          yDiff = self.screen(shipState.y) - oldY
          newpos = self.shipPositions[i].move(xDiff, yDiff)
          self.shipPositions[i] = newpos
          # rotate the ship by the desired amount and draw
          center = newpos.center

          # TODO: for a top-left origin, and a '-->' oriented ship.png
          # the rotation should exactly be reflected
          toRotate = -shipState.orientation

          rotated = rotate(self.ship, toRotate)
          newpos = rotated.get_rect(center=center)
          self._surface.blit(rotated, newpos)

          self.shipRotatedPositions.append(newpos)

        """
        self.angle -= 1 
        if self.angle <= -360:
          self.angle = 0 
        """
        
        # TODO: in this complicated statement only the "else" part matters.
        # All the "try" part is just a way to avoid the unclear blinking
        # that happened during the first display.update(self.shipRotatedPositions...)
        try:
          flag
        except:
          # executed only the first time
          flag = 1
          pygame.display.update()
        else: 
          # executed every time except the first
          pygame.display.update(self.shipRotatedPositions + oldRotatedPositions)
        t2 = time.time()
        #print "draw:", (t2-t1)*1000.0

        
        # update frame number on the status bar
        self._framecounter += 1
        self.frame_1_statusbar.SetStatusText("Frame %i" % self._framecounter, 1)



    def UpdateBackground(self, event):
        """
        Colors the background in self.bgColor.
        This make sure background is always there,
        even after switching tabs, moving windows above it,
        and so on
        """
        x,y = self.notebook_1_pane_2.GetSizeTuple()
        if x != self.lastScreenX or y != self.lastScreenY or self.lastZoom != self.zoom:
          # resize happened, update display size
          self._surface = pygame.display.set_mode((x,y))
          self._surface.fill(self.bgColor)
        self.lastScreenX, self.lastScreenY, self.lastZoom = x, y, self.zoom

        # configuration-specific callbacks (see setConfigurationOptions() )
        for hook in self.updateBackgroundHooks:
          # execute each callback in the array
          hook()

        pygame.display.update()



    def OnQuit(self, event): # wxGlade: MainGUIWindow.<event_handler>
        self.Close()# TODO: is it correct to call Close both from here and from Update()?


    ##########################################################
    # Hooks for UpdateBackground()
    ##########################################################
    def drawRRTHook(self):
      """This callback is being used only if there is 
      an RRT to be drawn. It is determined in setConfigurationOptions()
      """
      for i, agent in enumerate(self.simEnv.shipAgents):
        #agent = self.simEnv.shipAgents[0]
        color = COLORS[i % len(COLORS)]
        if isinstance(agent.strategy.navigationTactic, AgentRRTTactic):
          # if just entered the RRT mode, draw tree and copy data 
          # (data is needed for later deletion from GUI)
          if self.agent2RRTData[i] == None:
            # copy data
            self.agent2RRTData[i] = copy.deepcopy(agent.strategy.navigationTactic.rrtBuilder)
            # draw tree
            self.drawRRT(agent.strategy.navigationTactic.rrtBuilder, color)
        else: 
          # not in RRT mode, if first time, delete tree and reset data
          if self.agent2RRTData[i] != None:
            self.drawRRT(self.agent2RRTData[i], self.bgColor) # practically deleting the tree
            self.agent2RRTData[i] = None
            
            

    def drawRRT(self, RRTStrategy, color):
      rrt = RRTStrategy.tree
      nodes = rrt.nodes
      # for each node, draw an edge to its parent
      for node in nodes:
        nodeState = node.state
        x1, y1 = self.screen(nodeState.x), self.screen(nodeState.y)
        if node.parent != None:
          parentState = node.parent.state
          x2, y2 = self.screen(parentState.x), self.screen(parentState.y)
        else:
          x2, y2 = x1, y1
        pygame.draw.lines(self._surface, color, False, [(x1,y1), (x2,y2)], 1)
      # draw the goal state
      goal = RRTStrategy.goalPos
      x, y = self.screen(goal.x), self.screen(goal.y)
      x, y = int(round(x)), int(round(y))
      pygame.draw.circle(self._surface, color, (x, y), 4, 0)

    def drawStartEndHook(self):
        """This hook is used when we want to draw the
        start and goal states.
        """
        for i, agent in enumerate(self.simEnv.shipAgents):
          #agent = self.simEnv.shipAgents[0]
          color = COLORS[i % len(COLORS)]

          # draw the start state
          externalState = self.initialSimEnv.state.shipExternalStates[i]
          x, y = self.screen(externalState.x), self.screen(externalState.y)
          pygame.draw.rect(self._surface, color, pygame.Rect(x+2,y+2,4,4))

          # draw the goal state
          goal = agent.strategy.goalPos
          x, y = goal
          x, y = self.screen(x), self.screen(y)
          pygame.draw.circle(self._surface, color, (x, y), 4, 0)

    def drawPathHook(self):
        """
        This hook is used when we want to draw the
        waypoints of each agent's path, for all agents.
        """
        for i, agent in enumerate(self.simEnv.shipAgents):
          #agent = self.simEnv.shipAgents[0]
          color = COLORS[i % len(COLORS)]

          # draw the start state
          #externalState = self.initialSimEnv.state.shipExternalStates[i]
          #x, y = self.screen(externalState.x), self.screen(externalState.y)
          #pygame.draw.rect(self._surface, color, pygame.Rect(x+2,y+2,4,4))

          # draw the goal state
          for point in agent.strategy.getPath(): #Note: depends only on an agent's actual path
            x, y = point
            x, y = self.screen(x), self.screen(y)
            #pygame.draw.circle(self._surface, color, (x, y), 4, 0)
            #pygame.draw.circle(self._surface, color, (x, y), 10, 0)
            self._surface.blit(self.waypoint, (x, y))

    def drawObstacleHook(self):
      """This hook is used for drawing a specific obstacle."""
      # draw real obstacles, that are loaded from file
      color = GREEN3
      for obst in self.simEnv.state.sea.obstacles.getObstaclesList():
        points = obst.getBorder()
        scaledPoints = [(self.screen(x), self.screen(y)) for x,y in points]
        #pygame.draw.polygon(self._surface, color, scaledPoints)
        pygame.draw.lines(self._surface, color, False, scaledPoints, 3)

    def onlineHeuristicDivideHook(self):
      """
      Draws a red/green background to every ship depending on whether they are ready
      """
      shipStates = self.simEnv.state.shipExternalStates
      agent = self.simEnv.shipAgents[0] # TODO: using the first agent for that

      patroling = (agent.strategy.missionState == agent.strategy.PATROLING)
      # create a counter when starts patroling
      if patroling:
        self.readyToPatrolCounter += 1
      else:
        self.readyToPatrolCounter = 0

      # TODO: counting on one agent's information for now, assuming no msg loss
      reachedStart = agent.strategy.reachedStartPoint
      for i, agent in enumerate(self.simEnv.shipAgents):
        startPoint = agent.strategy.myStartingPatrolPosition
        if startPoint != None:
          x, y = int(round(self.screen(startPoint.x))), int(round(self.screen(startPoint.y)))
          if patroling: 
            # once patroling, color everything in green, wait 5 cycles, and delete points
            if self.readyToPatrolCounter < 5:
              pygame.draw.circle(self._surface, GREEN3, (x, y), 12, 0)
            if self.readyToPatrolCounter == 5:
              pygame.draw.circle(self._surface, self.bgColor, (x, y), 12, 0) #delete points (every time, not very efficient)
          elif i in reachedStart:
            # draw green circle
            pygame.draw.circle(self._surface, GREEN3, (x, y), 12, 0)
          else:
            # draw red circle
            pygame.draw.circle(self._surface, RED, (x, y), 12, 0)


    def trackingHook(self):
      """
      Call back for drawing when a ship is tracking another
      """
      for i, agent in enumerate(self.simEnv.shipAgents):
        if isinstance(agent.strategy, AgentStaticPatrolStrategy):
          # tracked ship

          # draw path
          for point in agent.strategy.getPath(): #Note: depends only on an agent's actual path
            x, y = point
            x, y = self.screen(x), self.screen(y)
            #pygame.draw.circle(self._surface, color, (x, y), 4, 0)
            #pygame.draw.circle(self._surface, color, (x, y), 10, 0)
            self._surface.blit(self.waypoint, (x, y))
        else: 
          # tracking ship
          
          # init recording variables
          try:
            self.lastTrackingPoints
          except:
            self.lastTrackingPoints = {}
          try:
            self.lastTrackingGoalPos
          except:
            self.lastTrackingGoalPos = {}

          # draw tracking point
          # always delete last tracking point
          try: # should always succeed except for the first time
            pygame.draw.circle(self._surface, self.bgColor, self.lastTrackingPoints[i], 12, 0)
          except Exception, e:
            pass
          try: # should always succeed except for the first time
            trackingPoint = agent.strategy.trackingPoint
            screenTrackingPoint = (int(round(self.screen(trackingPoint[0]))), int(round(self.screen(trackingPoint[1]))))
            pygame.draw.circle(self._surface, DODGERBLUE, screenTrackingPoint, 12, 0)
            # delete previous tracking point
            # record current tracking point as last tracking point
            self.lastTrackingPoints[i] = screenTrackingPoint
          except Exception, e:
            pass


          # if reached goal position - delete it
          try:  # should always succeed except for the first time
            goalPos = agent.strategy.navigationTactic.getGoalPos()
            screenGoalPos = (self.screen(goalPos[0]), self.screen(goalPos[1]))
            # always delete last rect
            try:
              lastTrackingGoalPos = self.lastTrackingGoalPos[i]
              rect = self.waypoint_dynamic.get_rect().move(lastTrackingGoalPos[0], lastTrackingGoalPos[1])
              self._surface.fill(self.bgColor, rect)
            except Exception, e:
                print e
            # draw target point (might be the same as the one just deleted)
            self._surface.blit(self.waypoint_dynamic, screenGoalPos)
            self.lastTrackingGoalPos[i] = screenGoalPos
          except Exception, e:
              print e
            



        # artificial (jpg) obstacles/images, just for demonstration purposes
#        x, y = self.obstacles.get_size()
#        self.scaledObstacle = pygame.transform.scale(self.obstacles, (int(self.screen(x)),int(self.screen(y))))
#        self._surface.blit(self.scaledObstacle, (self.screen(350),self.screen(400)))
#        #self._surface.blit(self.obstacles, (self.screen(500),self.screen(400)))

    def basicPatrolHook(self):
        """Just calls other hooks."""
        self.drawPathHook()
        #self.drawObstacleHook()

    def noopHook(self):
      pass

    def screen(self, coord):
      return coord / self.zoom



# end of class MainGUIWindow




def runGui(env, shipType, worldModel, strategy, numSteps, speed):
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame_1 = MainGUIWindow(None, -1, "")
    frame_1.initData(env, shipType, worldModel, strategy, numSteps, speed)
    app.SetTopWindow(frame_1)
    frame_1.Show()
    app.MainLoop()

"""
if __name__ == "__main__":
    runGui(None) # TODO: None only because runGui expects an argument, just in the meantime
"""
