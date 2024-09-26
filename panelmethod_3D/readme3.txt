cd scripts
mkdir helmet_demo
cd helmet_demo

Check 8 buildings in arena (881 ... 889)

python ../get_buildings.py 
=> *.npy
In case of error (*) try again to obtain (**)

python ../generate_buildings_buffer.py
(python ../generate_buildings.py)
=> *.obj

(1)
paraview *obj
Apply
Select all
Ctrl space
Group Datasets
Apply
Ctrl space
Normal Glyphs
Apply

=> Check VTK number

(2) option
../generate_bridge.py
Apply 
...
Transform
...
Save Data


vi ../change_normals.py
python ../change_normals.py

paraview *obj
idem (1)


vi ../experiment_helmet.py
("66" , ...)
("888" , )

python ../experiment_helmet.py

-------------------------------------------------------------------------------
(*)
NAT_CONNECT to Motive with 4 1 0 0
WARNING: Early return.  Marker count too high
ERROR: Early End of Data Frame Suffix Data
        No time stamp info available
...
KeyError: '881'
^CException ignored in: <module 'threading' from '/usr/lib/python3.10/threading.py'>
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1567, in _shutdown
    lock.acquire()
KeyboardInterrupt:

(**)
NAT_CONNECT to Motive with 4 1 0 0
/home/pprz/Projects/vto-demo/panelmethod_3D/scripts/voliere.py:94: RuntimeWarning: invalid value encountered in arccos
  angle = 2 * np.arccos(w)
Shutting down NATNET ...
shutdown called
shutting down
pprz@poule:~/Pr
