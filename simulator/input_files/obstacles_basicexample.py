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
# This file is a config file for obstacles in the sea.
# The result of executing this file 
# is an assignment of an object of
# type Obstacles to the variable 
# "obstacles". You can uncomment one of the 
# sections below, or add your own.
#
#############################################




#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
# In this file: A Basic Case - a few simple obstacles
# Any polygon could be supported, but currently we only 
# use 2-point straight lines. By using multiple of these,
# compound obstacles can be built
# the following obstacles create a tooth like obstacle.

import seaModels

#obstacle2 = seaModels.Obstacle( ((850, 850), (850, 900)) ) #(750, 800), (800, 800), (800, 850), (850, 850), (850, 900), (840, 900), (840, 860), (790, 860), (790, 810), (740, 810), (750, 800)) )
#obstacle3 = seaModels.Obstacle( ((850, 900), (900, 900)))# ((840, 900), (900, 900), (900, 910), (840, 910), (840, 900)) )
#obstacle4 = seaModels.Obstacle( ((900, 900), (900, 950)))#((900, 900), (900, 950), (890, 950), (890, 900), (900, 900)) )
#obstacle5 = seaModels.Obstacle( ((900, 950), (950, 950)) )#((890, 950), (950, 950), (950, 960), (890, 960), (890, 950)) )
#obstacle6 = seaModels.Obstacle( ((950, 950), (950, 1000)) )#((950, 950), (950, 1000), (940, 1000), (940, 950), (950, 950)) )
#obstacle7 = seaModels.Obstacle( ((950, 1000), (1000, 1000)) )#((1050, 1050), (1050, 1100), (1040, 1100), (1040, 1050), (1050, 1050)) )
#obstacle8 = seaModels.Obstacle( ((1000, 1000), (1000, 1050)) )#((1040, 1100), (1100, 1100), (1100, 1110), (1040, 1110), (1040, 1100)) )
#obstacle9 = seaModels.Obstacle( ((1000, 1050), (1050, 1050)) )#((1100, 1100), (1100, 1150), (1090, 1150), (1090, 1100), (1100, 1100)) )
#obstacle10 = seaModels.Obstacle( ((550, 550), (550, 600), (600, 600), (600, 550), (550, 550)) )
obstacle1 = seaModels.Obstacle( ( (950, 750), (1010, 800)) )
obstacle2 = seaModels.Obstacle( ( (1010, 800), (920, 905)) )
obstacle3 = seaModels.Obstacle( ( (920, 905), (1100, 1100)) )
obstacle4 = seaModels.Obstacle( ( (1100, 1100), (1200, 1000)) )
obstacle5 = seaModels.Obstacle( ( (1200, 1000), (1250, 1050)) )
obstacle6 = seaModels.Obstacle( ( (1250, 1050), (1050, 1250)) )
obstacle7 = seaModels.Obstacle( ( (1050, 1250), (750, 950)) )
obstacle8 = seaModels.Obstacle( ( (750, 950), (950, 750)) )
obstacles = seaModels.Obstacles( [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8] )

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
