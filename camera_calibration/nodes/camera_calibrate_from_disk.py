#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os.path
import sys
import cv
from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo

def main(args):
  from optparse import OptionParser
  parser = OptionParser()
  parser.add_option("-s", "--size", default="8x6", help="specify chessboard size as nxm [default: %default]")
  parser.add_option("-q", "--square", default=".108", help="specify chessboard square size in meters [default: %default]")
  options, args = parser.parse_args()
  size = tuple([int(c) for c in options.size.split('x')])
  dim = float(options.square)

  images = []
  for fname in args:
    if os.path.isfile(fname):
      img = cv.LoadImage(fname)
      if img is None:
        print("[WARN] Couldn't open image " + fname + "!", file=sys.stderr)
        sys.exit(1)
      else:
        print("[INFO] Loaded " + fname + " (" + str(img.width) + "x" + str(img.height) + ")")

      images.append(img)

  cboard = ChessboardInfo()
  cboard.dim = dim
  cboard.n_cols = size[0]
  cboard.n_rows = size[1]
  
  mc = MonoCalibrator([cboard])
  mc.cal(images)
  print(mc.as_message())

if __name__ == "__main__":
  print('This script is deprecated. Use tarfile calibration instead', file=sys.stderr)
  if len(sys.argv) >= 2:
    main(sys.argv[1:])

