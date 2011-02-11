#!/usr/bin/python

PKG = 'camera_calibration' # this package name
import roslib; roslib.load_manifest(PKG)

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
        print >> sys.stderr, "[WARN] Couldn't open image " + fname + "!"
        sys.exit(1)
      else:
        print "[INFO] Loaded " + fname + " (" + str(img.width) + "x" + str(img.height) + ")"

      images.append(img)

  cboard = ChessboardInfo()
  cboard.dim = dim
  cboard.n_cols = size[0]
  cboard.n_rows = size[1]
  
  mc = MonoCalibrator([cboard])
  mc.cal(images)
  print mc.as_message()

if __name__ == "__main__":
  print >> sys.stderr, 'This script is deprecated. Use tarfile calibration instead'
  if len(sys.argv) >= 2:
    main(sys.argv[1:])

