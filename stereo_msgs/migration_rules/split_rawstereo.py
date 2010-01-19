#!/usr/bin/env python

PKG = 'stereo_msgs'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosrecord
import fileinput
import os

def split_rawstereo(inbags):
  for b in inbags:
    print "Trying to migrating file: %s"%b
    outbag = b+'.tmp'
    rebag = rosrecord.Rebagger(outbag)
    try:
      for (topic, msg, t) in rosrecord.logplayer(b, raw=False):
        if topic == 'stereo/raw_stereo' or topic == '/stereo/raw_stereo':
          m = msg.left_image
          m.header.frame_id = msg.header.frame_id
          m.header.stamp = msg.header.stamp
          rebag.add('stereo/left/image_raw', m, t, raw=False)
          m = msg.right_image
          m.header.frame_id = msg.header.frame_id
          m.header.stamp = msg.header.stamp
          rebag.add('stereo/right/image_raw', m, t, raw=False)
          m = msg.left_info
          m.header.frame_id = msg.header.frame_id
          m.header.stamp = msg.header.stamp
          rebag.add('stereo/left/camera_info', m, t, raw=False)
          m = msg.right_info
          m.header.frame_id = msg.header.frame_id
          m.header.stamp = msg.header.stamp
          rebag.add('stereo/right/camera_info', m, t, raw=False)
        else:
          rebag.add(topic, msg, t, raw=False)
      rebag.close()
    except rosrecord.ROSRecordException, e:
      print " Migration failed: %s"%(e.message)
      os.remove(outbag)
      continue

    oldnamebase = b+'.old'
    oldname = oldnamebase
    i = 1
    while os.path.isfile(oldname):
      i=i+1
      oldname = oldnamebase + str(i)
    os.rename(b, oldname)
    os.rename(outbag, b)
    print " Migration successful.  Original stored as: %s"%oldname

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 2:
    split_rawstereo(sys.argv[1:])
  else:
    print "usage: split_rawstereo.py bag1 [bag2 bag3 ...]"


