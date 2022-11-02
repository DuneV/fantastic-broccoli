#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


def get_positions():
    posiciones = [[123,456], [789, 987], [654, 321]]

    # TODO: agregar funcion de filtro

    poses_list = []
    for i in posiciones:
        pose=Pose()
        pose.position.x = float(i[0])
        pose.position.y = float(i[1])
        poses_list.append(pose)
    pose_array = PoseArray()
    pose_array.header.frame_id ='1'
    pose_array.poses=poses_list
    return pose_array


def main():
    rospy.init_node("limpieza")
    pub = rospy.Publisher('posiciones_sucias', PoseArray, queue_size=10)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher created")
    called = False


    while not rospy.is_shutdown():
        if not called:
            called = True
        pub.publish(get_positions())
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
