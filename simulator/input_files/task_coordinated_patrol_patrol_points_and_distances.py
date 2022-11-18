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



patrolPoints=[(300, 500), (750, 290), (1090, 655), (1390, 655), (1866, 505), (1866, 995), (1390, 845), (1090, 845), (750, 1210), (300, 1000)]
edgeLengths={((1390, 845), (1866, 995)): [146, 146, 146, 146, 146, 146], ((300, 500), (750, 290)): [166, 147, 147, 147, 147, 147], ((750, 1210), (300, 1000)): [160, 160, 160, 160, 160], ((1090, 845), (1090, 655)): [96, 77, 97, 77, 97, 77, 97, 77, 97, 77, 97, 77, 97], ((1090, 655), (750, 290)): [149, 149, 149, 149, 149], ((1866, 995), (1390, 845)): [159, 160, 160, 160, 160], ((1390, 845), (1390, 655)): [75, 84, 75, 84, 75, 84, 75, 84, 75, 84, 75, 84, 75], ((1090, 655), (1390, 655)): [73, 74, 72, 71, 77, 72, 71, 72, 77, 71, 72, 77, 71, 72, 71, 77, 72, 71, 77, 72], ((1866, 505), (1866, 995)): [150, 150, 150, 150, 150, 150], ((300, 1000), (750, 1210)): [149, 147, 147, 147, 147, 147], ((1090, 845), (1390, 845)): [76, 76, 72, 72, 77, 73, 72, 73, 77, 72, 73, 77, 72, 73, 72, 77, 73, 72, 77, 73], ((1390, 655), (1390, 845)): [76, 82, 74, 82, 74, 82, 74, 82, 74, 82, 74, 82, 74, 82], ((300, 500), (300, 1000)): [150, 151, 151, 151, 151], ((1090, 845), (750, 1210)): [149, 149, 149, 149, 149], ((1866, 505), (1390, 655)): [161, 160, 160, 160, 160], ((750, 1210), (1090, 845)): [147, 149, 149, 149, 149, 149], ((300, 1000), (300, 500)): [150, 150, 150, 150, 150], ((750, 290), (300, 500)): [161, 160, 160, 160, 160], ((1090, 655), (1090, 845)): [98, 78, 97, 78, 97, 78, 97, 78, 97, 78, 97, 78, 97], ((1390, 655), (1090, 655)): [474, 517, 511, 473, 512, 516, 473, 512, 516, 473, 512, 473, 516, 512, 473, 516, 512, 473], ((750, 290), (1090, 655)): [156, 148, 148, 148, 148, 148], ((1390, 845), (1090, 845)): [472, 513, 512, 472, 512, 513, 472, 512, 513, 472, 512, 472, 513, 512, 472, 513, 512, 472], ((1866, 995), (1866, 505)): [150, 151, 151, 151, 151, 151], ((1390, 655), (1866, 505)): [146, 145, 145, 145, 145, 145]}
