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
env ROS_MASTER_URI="$rosUri" GAZEBO_MASTER_URI="$gazeboUri" roslaunch comp_cov_sim comp_cov_sim.launch gui:=false --port=$rosPort > /dev/null 2>&1 &
done

