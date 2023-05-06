#! /usr/bin/env python

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from  tf.transformations import quaternion_from_euler

if __name__ == "__main__":
    rospy.init_node("static_tf_pub_p")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tfs = TransformStamped()
    tfs.header.frame_id = "world"
    tfs.header.stamp = rospy.Time.now()
    tfs.header.seq = 101
    tfs.child_frame_id = "camera_link"
    tfs.transform.translation.x = 0.0
    tfs.transform.translation.y = 0.0
    tfs.transform.translation.z = 0.0

    qtn = quaternion_from_euler(-1.5708, -1.5708, 0)
    # qtn = quaternion_from_euler(0, 0, 0)

    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]

    broadcaster.sendTransform(tfs)
    rospy.spin()



