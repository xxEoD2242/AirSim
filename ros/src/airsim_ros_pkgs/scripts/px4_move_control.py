import rospy
from airsim_ros_pkgs.srv import SetLocalPosition

if __name__ == '__main__':
    rospy.init_node('move_drone_node')
    parent_frame = rospy.get_param('~parent_frame', 'world')
    child_frame = rospy.get_param('~child_frame','PX4')
    pose_topic = rospy.get_param('~pose_topic','/airsim_node/PX4/front_center_custom/pose')
    odom_topic = rospy.get_param('~odom_topic','/airsim_node/PX4/odom_local_enu')
    goal_topic = rospy.get_param('~goal_topic', "/airsim_node/local_position_goal")
    service_name = '/airsim_node/local_position_goal'
    rospy.wait_for_service(service_name)
    set_local_position = rospy.ServiceProxy(service_name, SetLocalPosition)
    
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        
        print("Move the drone by x, y, z (meters) and yaw (degrees).\n")
        input_valid = False
        while not input_valid:
            try:
                x_pos = input("X Position: \t")
                assert isinstance(float(x_pos), float)
                y_pos = input("Y Position: \t")
                assert isinstance(float(y_pos), float)
                z_pos = input("Z Position: \t")
                assert isinstance(float(z_pos), float)
                yaw = input("Yaw: \t")
                assert isinstance(float(yaw), float)
                input_valid = True
            except AssertionError:
                print("This input was not correct!")

        print("Goal input is valid: \n\n Executing...")
        """
        TODO: Make a custom message structure to handle this
        goal = SetLocalPosition()
        goal.x = float(x_pos)
        goal.y = float(y_pos)
        goal.z = float(z_pos)
        goal.yaw = float(yaw)
        goal.vehicle_name = child_frame
        """

        try:
            # This calls the service with a message strucutre that is defined
            # in the SetLocalPosition.srv
            resp1 = set_local_position(x_pos, y_pos, z_pos, yaw, child_frame)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        print("Vehicle made it!")
        rate.sleep()