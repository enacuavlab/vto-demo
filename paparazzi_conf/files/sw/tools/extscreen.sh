#!/bin/sh
gst-launch-1.0 udpsrc port=5700 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! tee name=t t. ! queue ! videoconvert ! autovideosink t. ! queue ! videoconvert ! autovideosink &
while [ `wmctrl -l | grep -c "gst-launch-1.0"` -eq 0 ]
do 
  sleep 1 
done
if [ `wmctrl -l | grep -c "gst-launch-1.0"` -eq 2 ] 
then
  x=1920
  for w in `wmctrl -l | grep "gst-launch-1.0" | awk '{print $1}'`
  do
    echo $w
    wmctrl -i -r $w -e 0,$x,0,-1,-1
    x=0
  done 
fi
