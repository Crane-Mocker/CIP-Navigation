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





################################################################
# The whole file is taken from:
# http://refactormycode.com/codes/1114-line-line-intersection-test
################################################################

# vector class from pygame cookbook http://www.pygame.org/wiki/2DVectorClass
from vec2d import *
import math

def lineline(A,B,C,D):
    """ Line-line intersection algorithm,
            returns point of intersection or None
    """
    # ccw from http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
    def ccw(A,B,C):
        return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x)
    if ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D):
        # formula from http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
        ua =    float(((D.x-C.x)*(A.y-C.y))-((D.y-C.y)*(A.x-C.x)))/ \
                float(((D.y-C.y)*(B.x-A.x))-((D.x-C.x)*(B.y-A.y)))
        ub =    float(((B.x-A.x)*(A.y-C.y))-((B.y-A.y)*(A.x-C.y)))/ \
                float(((D.y-C.y)*(B.x-A.x))-((D.x-C.x)*(B.y-A.y)))
        return Vec2d(   A.x+(ua*(B.x-A.x)), \
                        A.y+(ua*(B.y-A.y)))
    return None

def distance(ax, ay, bx, by):
  return math.sqrt((ax - bx)**2 + (ay - by)**2)

def isBetween(ax, ay, bx, by, cx, cy):
  return distance(ax, ay, cx, cy) + distance(cx, cy, bx, by) == distance(ax, ay, bx, by)


def linelinedemo():
    """ Graphical demo showing the line line intersection algorithm.
            Click and hold left mouse button to place first line,
            right mouse button places second line.
            A white circle will be draw at the point of intersection.
    """
    import pygame
    from pygame.locals import QUIT,KEYDOWN,MOUSEBUTTONDOWN,MOUSEBUTTONUP

    pygame.init()
    screen = pygame.display.set_mode((256,256))
    clock = pygame.time.Clock()

    A,B,C,D = None,None,None,None
    running = True
    while running:
        for event in pygame.event.get():
            if event.type in (QUIT, KEYDOWN):
                running = False
            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                A = Vec2d(pygame.mouse.get_pos())
                B = None
            elif event.type == MOUSEBUTTONDOWN and event.button == 3:
                C = Vec2d(pygame.mouse.get_pos())
                D = None
            elif event.type == MOUSEBUTTONUP and event.button == 1:
                B = Vec2d(pygame.mouse.get_pos())
            elif event.type == MOUSEBUTTONUP and event.button == 3:
                D = Vec2d(pygame.mouse.get_pos())

        screen.fill((0,0,0))

        if A is not None:
            endpos = B or pygame.mouse.get_pos()
            pygame.draw.line(screen, (255,0,0), A, endpos)

        if C is not None:
            endpos = D or pygame.mouse.get_pos()
            pygame.draw.line(screen, (0,255,0), C, endpos)

        if A and B and C and D:
            v = lineline(A,B,C,D)
            if v:
                pygame.draw.circle(screen, (255,255,255), (int(v.x),int(v.y)), 5, 1)

        pygame.display.flip()
        clock.tick(100)

if __name__=="__main__":
    linelinedemo()

