#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
import copy
import moveit_msgs.msg


def move_group_python_interface(plan_type):

    print "================== Kinova Motion Planning Setup =================="
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_move_group', anonymous=True)  # need todo

    kinova = moveit_commander.RobotCommander()
    kinova_groups = kinova.get_group_names()

    rospy.sleep(1)

    print "================== Got all valid Groups: =================="
    print kinova_groups

    if group_name not in kinova_groups:
        raise NameError("The group name given is not in valid Groups")

    print "================== Group: %s will be planed. ==================" % group_name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    print "================== Current End Effector Position: =================="
    print move_group.get_current_pose().pose.position

    print "================== Current End Effector Orientation: =================="
    print move_group.get_current_pose().pose.orientation

    if plan_type == "pose":

        # Create a pose structure
        pose_target = geometry_msgs.msg.Pose()

        x, y, z = 0.252743313299, -0.256609270659, 0.506962197378
        ox, oy, oz, ow = 0.682329904394, -0.228853292756, 0.68849070917, 0.0896248589985
        pose_target.orientation.x = ox
        pose_target.orientation.y = oy
        pose_target.orientation.z = oz
        pose_target.orientation.w = ow
        pose_target.position.x = x  # move_group.get_current_pose().pose.position.x
        pose_target.position.y = y  # move_group.get_current_pose().pose.position.y
        pose_target.position.z = z  # move_group.get_current_pose().pose.position.z

        move_group.set_pose_target(pose_target)

        #move_group.set_planner_id('RRTstarkConfigDefault')

        # move_group.set_planning_time(15)
        # move_group.set_num_planning_attempts(500)

        my_plan = move_group.plan()

        move_group.go()

    elif plan_type is "cartesian":

        waypoints = []

        # start with the current pose
        waypoints.append(move_group.get_current_pose().pose)

        # first way point
        wpose = geometry_msgs.msg.Pose()

        wpose.orientation.x,\
        wpose.orientation.y,\
        wpose.orientation.z,\
        wpose.orientation.w = 0.021535, -0.0036272, -0.68021, 0.73269  # put your orientation here

        wpose.position.x,\
        wpose.position.y,\
        wpose.position.z = 1.0536, 0.024026, 0.30  # put your position here
        waypoints.append(copy.deepcopy(wpose))

        # second way point

        wpose.orientation.x, \
        wpose.orientation.y, \
        wpose.orientation.z, \
        wpose.orientation.w = 0.021535, -0.0036272, -0.68021, 0.73269  # put your orientation here

        wpose.position.x, \
        wpose.position.y, \
        wpose.position.z = 1.0536, 0.024026, 0.08  # put your position here
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                                            waypoints,  # waypoints to follow
                                                            0.01,       # eef_step
                                                            0.0)        # jump_threshold

        print "============ Waiting while RVIZ displays plan..."
        rospy.sleep(5)

        move_group.execute(plan, wait=True)

    else:
        raise TypeError("A invalid Type was given!")


if __name__ == '__main__':
    try:
        group_name = 'arm'
        move_group_python_interface('pose')
    except rospy.ROSInterruptException:
        pass
