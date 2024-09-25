-------------------------------------------------------------------------------
sudo apt-get install paraview
sudo apt-get install libcgal-dev
sudo apt-get install libeigen3-dev
pip3 install pygalmesh

cd panelmethod_3D

sudo apt-get install cmake

# unzip DJITelloPy-master.zip

unzip DJITelloPy-path_plan.zip
cd  DJITelloPy-path_plan
pip install .

# unzip mocap-Avivation_24.zip

unzip PGFlow3D-boost_geo.zip

tar xvf boost_geometry.tar -C PGFlow3D-boost_geo/thirdparty/

cd PGFlow3D-boost_geo/thirdparty
git clone https://github.com/pybind/pybind11.git
git clone https://gitlab.com/libeigen/eigen.git
git clone https://github.com/guybrush77/rapidobj.git
cd PGFlow3D-boost_geo
pip install .

# unzip PGFlow3D-master.zip

# git clone https://github.com/enac-drones/PGFlow3D.git --recursive
# cd PGFlow3D
# pip install .

pip install pandas
pip install scipy --upgrade
pip install numpy==1.26.4
pip install shapely==2.0.4
pip install pybullet
pip install tqdm

-------------------------------------------------------------------------------
pip install requests

-------------------------------------------------------------------------------
patch voliere.py:
      if not rigid_body.tracking_valid:
            continue
      i = str(rigid_body.id_num)
to
      i = str(rigid_body.id_num)
      if not rigid_body.tracking_valid:
        self.vehicles[i].valid = False
        continue


patch experiment_helmet.py:
  while time.time() - sim_start_time < 180:
to
  validtrack = 0
  validtrackmax = FREQ * 3
  try 
  ...
  while ((time.time() - sim_start_time < 30) and (validtrack < validtrackmax)):
