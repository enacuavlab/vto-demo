cd scripts
mkdir helmet_demo
cd helmet_demo

Check 9 buildings in arena (881 ... 882)
python ../get_buildings.py 
=> *.npy

python ../generate_buildings.py
(python ../generate_buildings_buffer.py)
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
