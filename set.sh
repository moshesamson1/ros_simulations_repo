# get amount of iterations
echo "Number of Sessions:"
read amount

# running X terminals, each running a new instance of simulation
baseRosPort=11312
baseGazeboPort=11346

echo "source catkin..."
source ~/catkin_ws/devel/setup.sh
source ~/catkin_ws/devel/setup.bash

for((counter=0; counter<$amount; counter++))
do
rosPort=$((baseRosPort+counter))
rosUri="http://localhost:$rosPort"
gazeboPort=$((baseGazeboPort+counter))
gazeboUri="http://localhost:$gazeboPort"


echo "Session number $counter, ros path: $rosUri, gazebo path: $gazeboUri"
(rosPort=${rosPort} gazeboPort=${gazeboPort} ROS_MASTER_URI=http://localhost:${rosPort} GAZEBO_MASTER_URI=http://localhost:${gazeboPort} sh -c 'roslaunch comp_cov_sim comp_cov_sim.launch --port=${rosPort} > ${rosPort}_${gazeboPort}_output 2>&1; bash analize.sh > ${rosPort}_${gazeboPort}_results' ) & 

done

