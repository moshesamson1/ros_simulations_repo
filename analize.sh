ros_port=$(echo $ROS_MASTER_URI | grep -E '[0-9]+' --only-matching)
gazebo_port=$(echo $GAZEBO_MASTER_URI | grep -E '[0-9]+' --only-matching)
robot0_file=src/${ros_port}_${gazebo_port}_robot_0_positions
robot1_file=src/${ros_port}_${gazebo_port}_robot_1_positions

robot_0_data=$(cat $robot0_file)
robot_1_data=$(cat $robot1_file)

robot_0_firsts=0
robot_1_firsts=0
for i in {0..23}
do
	for j in {0..23}
	do
		# analyze the current location, and add counter to the earlier visitor
		robot_0_time=$(echo "$robot_0_data" | grep $i,$j | grep -E '[0-9]+\.[0-9]+' -o)
		robot_1_time=$(echo "$robot_1_data" | grep $i,$j | grep -E '[0-9]+\.[0-9]+' -o)
		
		if [[ -z $robot_0_time || -z $robot_1_time ]]
		then
			continue
		fi
	
		if (( $(echo "$robot_0_time < $robot_1_time" |bc -l) ))
		then
			robot_0_firsts=$((robot_0_firsts+1))
		else
			robot_1_firsts=$((robot_1_firsts+1))
		fi
	done
done

echo "robot_0 got $robot_0_firsts out of 1024"
echo "robot_1 got $robot_1_firsts out of 1024"
