### In order to test this version the following list of commands is provided.  

First of all, it is needed to go in the workspace folder, build with `colcon build` and source the environment.  

Then the QGroundControl application should be executed.  

  - To test this version a simulation from PX4-Autopilot is employed. From the corresponding folder: 
  
      `make px4_sitl gz_custom_drone`

  - Then the XRCE-DDS agent is run (depending on which folder it is in):  
      `/home/user/ros2_ws/src/aerial_robotics#` `./DDS_run.sh`
  - Then a script is run from the folder in which the data to plot will be saved:   
      `/home/user/ros2_ws/src/bag_files#` `./run_bag.sh`   
      In particular this script will use the "ros2 bag" utility to record data from the folowing topics:   
      `/fmu/out/vehicle_local_position`  (for point 2)  
      `/fmu/out/manual_control_setpoint` (for point 2)   
      `/fmu/out/actuator_outputs`   (for point 1)
  - Now to test the new functionalities the force_landing node is run with the proper executable (from inside the aerial robotics folder):    
      `ros2 run force_landing force_landing`
  - At last, from the folder bag_files, a python script is executed to plot the varying altitude of the drone, the stick throttle provided by the pilot control and the actuators' outputs:   
      `/home/user/ros2_ws/src/bag_files#` `python3 plot_data.py`


#### Overall changes apported for the first point(to include later in the report):
- 1a) Using the x500 template the custom drone was created (this required the creation of two new folders in `PX4-Autopilot/Tools/simulation/gz/models/` in which- for both the folders- the `model.config` and the `model.sdf` were added, in addition to some custom texture added in the proper meshes folder); specifically an additional weight was added to the base link (and the inertia paramters were modified accordingly) to account for a virtual load (which is visualized as a red box on TOP of the drone (still need to check for correct positioning).   Then the related drone airframe file was created and placed in the `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes` folder and the relative cmakelists file was modified to account for this new airframe. This ensures that the simulation script corresponding to the custom drone can be later run.
- 1b) For this point it was needed to look for the actuator outputs msg that is not published on the xrce bridge by default. To enable this functionality the `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` file was modified adding this additional published topic.   Since the 2nd point was tackled earlier in which a ros2 bag was used, the same procedure is applied here. A bag is simply a ros2 tool that sneaks on a publishing topic and saves the data. So instead of using additional logic, a bash script was created to set the bag to listen to the 3 topics altogether (the one needed for this assignment and the two needed for point 2). Also a yaml file containing the relative quality_of_service options was created to get the most of the data published (by default the topics don't publish with best effort q_o_s)-> see `bag_files/qos_override.yaml`.   At last a python script was created to plot all the data (even in this case the python script refers to the other 2 topics required in the second point too): `bag_files/plot_data.py`


