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
=> bridge.obj
mv bridge.obj new_building_8_buff.obj


paraview *.obj
Apply 
select new_building_8_buff.obj
right click - Add Filter - Alphabetical - Transform
Move / resize
Apply
select Transform1
Menu File - Save Data - 
File name : new_building_8_buff.obj
Overwrite 


vi ../change_normals.py
python ../change_normals.py

paraview *obj
idem (1)


vi ../experiment_helmet.py
("66" , ...)
("888" , )

python ../experiment_helmet.py
