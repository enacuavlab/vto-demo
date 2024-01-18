#!/usr/bin/env python3
import sys
sys.path.insert(0,"/home/pprz/Projects/vto-natnet/common")
from NatNetClient import NatNetClient

import csv
import argparse
import time
import math
import copy
from collections import deque
import numpy as np

buildingDic = {}
loop = True

#--------------------------------------------------------------------------------
#
# ./exec_getbuildings.py -i material/'Take 2022-11-08 08.26.58 AM.csv' -o material/outputfromtake.csv
# or
# ./exec_getbuildings.py -o material/outputfromnatnet.csv
#
#--------------------------------------------------------------------------------

# This program gets top surface as building rigidbody (Building_xxx)

# Option 1: From Optitrack Motive tracking data as a csv extracted file
#   Export extracting data#

#     Header information
#     Markers
#
#     Axis Convention : Custom
#       X Axis : Left(X+)
#       Y Axis : Backward(Z-)
#       Z Axis : Up(Y+)

# Option 2: From Optitrack Natnet markers live positions 
#   This programs uses MoCapData.py from natnet SDK 4.0 (windows package)
#   This program gets data from Motive Server v3, with following settings:
#   Local Interface : 192.168.1.231
#   Transmission Type : Multicast
#   Labeled Markers : OFF
#   Unlabeled Markers : OFF
#
#   Asset Markers : ON
#   Rigid Bodies : ON or OFF
#
#   Up Axis : Z-Axis
#
#   Command Port : 1510
#   Data Port : 1511
#   Multicast Interface : 239.255.42.99

#--------------------------------------------------------------------------------
class LightBuilding():
  def __init__(self,name,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.name = name
    self.vertices = vertices

#--------------------------------------------------------------------------------
def write_verts(filename,buildingList):
  with open(filename,'w+') as out:
    for elt in buildingList:
      stringvalues = elt.name
      for vert in elt.vertices:
        stringvalues = stringvalues + ("[%.8f %.8f %.8f]" % (vert[0],vert[1],vert[2]))
      out.write(stringvalues+'\n')

#--------------------------------------------------------------------------------
def sort_verts(buildingList):
  for item1 in buildingDic.items():
    pts = np.empty((len(item1[1].items()),2))
    for i,item2 in enumerate(item1[1].items()): pts[i] = np.array(item2[1][0:2])
    angles = []
    (x0,y0)=(0,0)
    for i in pts: (x0,y0) = (i[0]+x0,i[1]+y0)
    (x0,y0) = (x0/len(pts),y0/len(pts))
    for i in pts:
      (dx,dy) = (i[0]-x0,i[1]-y0)
      angles.append(math.degrees(math.atan2(dy,dx)))
    tmp = [copy.copy(x) for y,x in sorted(zip(angles,pts), key=lambda pair: pair[0])]
    for i,item in enumerate(tmp):  pts[i] = item
    verts = np.empty((len(item1[1].items()),3))
    for i,item2 in enumerate(item1[1].items()): verts[i] = (pts[i][0],pts[i][1],4.2)
    b = LightBuilding(item1[0],verts)
    buildingList.append(b)

#--------------------------------------------------------------------------------
def csvfile_parse(inputfile):
  global buildingDic

  with open(inputfile, newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    for j in range(3): dummy = next(csvreader)
    fields = next(csvreader)
    for j in range(3): dummy = next(csvreader)
    values = next(csvreader)
    buildings = {}
    for i in range(2,len(fields),3):
      buildingName,markerName  = fields[i].split(':')
      floatLst=[float(x)/1000.0 for x in values[i:i+3]]
      if (not buildingDic) or (not buildingName in buildingDic): buildingDic[buildingName]={markerName:floatLst}
      buildingDic[buildingName].update({markerName:floatLst})

#------------------------------------------------------------------------------
def receiveRigidBodyMarkerSetList( rigid_body_data, marker_set_data, stamp ):
  global buildingDic
  global loop
  loop = False

#  for rigid_body in rigid_body_data.rigid_body_list:
#    print(str(rigid_body.id_num))

  for marker_data in marker_set_data.marker_data_list:
    model_name, separator, remainder = marker_data.model_name.partition( b'\0' )
    buildingName = model_name.decode( 'utf-8' )
    if not buildingName.startswith('Building_'):
      continue
    marker_count = len(marker_data.marker_pos_list)
    for j in range(marker_count):
      markerName = str(j)
      pos = marker_data.marker_pos_list[j]
      floatLst=[float(x) for x in pos]
      if (not buildingDic) or (not buildingName in buildingDic): buildingDic[buildingName]={markerName:floatLst}
      buildingDic[buildingName].update({markerName:floatLst})

#--------------------------------------------------------------------------------
if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_csvtakefile')
  parser.add_argument( '-o', '--output_csvfile')
  args = parser.parse_args()
  buildingList = []

  if args.output_csvfile:
    if args.input_csvtakefile:
      print("Building from take csv file")
      csvfile_parse(args.input_csvtakefile)
      sort_verts(buildingList)
      write_verts(args.output_csvfile,buildingList)
      exit(0)

  print("Searching for live multicast buildings")
  natnet = NatNetClient()
  natnet.set_server_address("192.168.1.240")
  natnet.set_client_address('0.0.0.0')
  natnet.set_print_level(0)  # 1 to print all frames
  natnet.rigid_body_marker_set_list_listener = receiveRigidBodyMarkerSetList
  
  try:
    is_running = natnet.run()
    if not is_running:
      print("Natnet error: Could not start streaming client.")
      exit(-1)
    time.sleep(1)
    if not natnet.connected():
      print("Natnet error: Fail to connect to natnet")
      exit(-1)
  
    while(loop): time.sleep(1)
    sort_verts(buildingList)
    write_verts(args.output_csvfile,buildingList)
    exit(0)
  
  except (KeyboardInterrupt, SystemExit):
    print("Shutting down natnet interfaces...")
    natnet.shutdown()
  except OSError:
    print("Natnet connection error")
    natnet.shutdown()
    exit(-1)
