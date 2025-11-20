### In order to test this version the following list of commands is provided.  

First of all, it is good practice to go in the workspace folder, build with `colcon build` and source the environment.  

Then the QGroundControl application should be executed.  

  - To test this version a simulation from PX4-Autopilot is employed. From the corresponding folder: 
  
      `make px4_sitl gz_x500`

  - Then the XRCE-DDS agent is run (depending on which folder it is in):  
      `/home/user/ros2_ws/src/aerial_robotics#` `./DDS_run.sh`
  - Then a script is run from the folder in which the data to plot will be saved:   
      `/home/user/ros2_ws/src/bag_files#` `./run_bag.sh`   
      In particular this script will use the "ros2 bag" utility to record data from the folowing topics:   
      `/fmu/out/vehicle_local_position`    
      `/fmu/out/manual_control_setpoint`
  - Now to test the new functionalities the force_landing node is run with the proper executable (from inside the aerial robotics folder):    
      `ros2 run force_landing force_landing`
  - At last, from the same folder (bag_files), a python script is executed to plot the varying altitude of the drone and the stick throttle provided by the pilot control:   
      `/home/user/ros2_ws/src/bag_files#` `python3 plot_data.py`
