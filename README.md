# ros_simulations_repo

Single Startup process:

  cd ~/catkin_ws
  source devel/setup.bash
  source devel/setup.sh
  cd src
  roslaunch comp_cov_sim comp_cov_sim.launch gui:=true (or false, for no gui) 
  

Setting the real time factor:
in the empty.world file, change the real time factor.

Setting multiple ros masters and gazebo servers:
  in each terminal:
    export $ROS_MASTER_URI=http://localhost:11311 (or any other port)
    export $GAZEBO_MASTER_URI=http://localhost:11345 (or any other port)
    in the ROSLAUNCH call, add "--port=11311" in the end, for the ros master uri. 

Running simulations via script:
  Run "./Set.sh", and enter the desired number of simulations 
