# Python package requested

sudo apt-get install python3-pyclipper python3-shapely



# OPTITRACK MOTIVE STREAM CONFIGURATION
#   Asset Markers : ON 
#   Rigid Bodies : ON of OFF
#
# Select only BUILDINGS_xxx
#

./exec_getbuildings.py -i /tmp/'Take 2022-11-15 03.10.48 PM.csv' -o tmp/outputfromtake.csv
or
./exec_getbuildings.py -o /tmp/outputfromnatnet.csv



./exec_genmatrix.py -i /tmp/outputfromtake.csv -o /tmp/outputfromtake.json
or
./exec_genmatrix.py -i /tmp/outputfromnatnet.csv -o /tmp/outputfromnatnet.json




./exec_display.py -i /tmp/outputfromtake.json
or
./exec_display.py -i /tmp/outputfromnatnet.json




# OPTITRACK MOTIVE STREAM CONFIGURATION
#   Asset Markers : ON or OFF
#   Rigid Bodies : ON
#
# Select rigibodies : HELMET_888 and TELLO_xxx
#

./exec_run.py -i /tmp/outputfromtake.json
or
./exec_run.py -i /tmp/outputfromnatnet.json



gst-launch-1.0 -v udpsrc port=11111 caps="video/x-h264, stream-format=(string)byte-stream" ! decodebin ! videoconvert ! autovideosink sync=false


gst-launch-1.0 -v udpsrc port=11115 caps="video/x-h264, stream-format=(string)byte-stream" ! decodebin ! videoconvert ! autovideosink sync=false
gst-launch-1.0 -v udpsrc port=11116 caps="video/x-h264, stream-format=(string)byte-stream" ! decodebin ! videoconvert ! autovideosink sync=false
...

######################################################################################
# USAGE !!

- Target should move outside buildings perimeter
- Building shoud be at least at 1 m from others
- If helmet is upside down for at least 1 sec. drones are landed

######################################################################################
