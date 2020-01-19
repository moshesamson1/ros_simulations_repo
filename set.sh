# get amount of iterations
echo "Number of Sessions:"
read amount

# running X terminals, each running a new instance of simulation
baseRosPort=11311
baseGazeboPort=11345
for((counter=0; counter<$amount; counter++))
do
rosPort=$((baseRosPort+counter))
rosUri="http://localhost:$rosPort"
gazeboPort=$((baseGazeboPort+counter))
gazeboUri="http://localhost:$gazeboPort"

echo "opening terminal number: $counter, ros path: $rosUri, gazebo path: $gazeboUri"
gnome-terminal -x bash -c '\n
 source ~/catkin_ws/devel/setup.sh;\
 source ~/catkin_ws/devel/setup.bash;\
 export ROS_MASTER_URI="$1";\
 export GAZEBO_MASTER_URI="$2";\
 roslaunch comp_cov_sim comp_cov_sim.launch gui:=false --port="$3"; exit' sh $rosUri $gazeboUri $rosPort
done

