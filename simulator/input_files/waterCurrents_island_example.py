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
# This file is a config file for water currents. 
# The result of executing this file 
# is an assignment of an object of
# type WaterCurrents to the variable 
# "waterCurrents". You can define similar
# files, and set them in the task definition 
# language file, to be used in your task
#
#############################################



import seaModels

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# 
# # ISLAND WATER 
# # ~~~~~~~~~~~~
#
islandPoints =  [(640 ,990), (1150,830), (1310,1130), (1240,1580), (1190,1960), (1400, 1740), (1640, 1580), (1860,1510), (2140, 1420), (2640, 1260), (2640, 1620), (2860, 1680), (3140, 1700), (3440, 1980), (3940, 1820), (4260, 1480), (4480, 1120), (4640, 720), (4960, 300), (5240, 640), (5200, 940), (4880, 1320), (4850, 1700), (4480, 2020), (4000, 2240), (3340, 2500), (2860, 2500), (2720, 2260), (2580, 2120), (2080, 2320), (1840,2290), (1540, 2300), (1240, 2500), (750, 2640), (480, 2140), (660, 1660)]
 
waterCurrents = seaModels.IslandWaterCurrents(islandPoints)
# 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


