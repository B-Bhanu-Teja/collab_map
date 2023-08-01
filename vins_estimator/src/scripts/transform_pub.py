#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('world_body_tf_publisher')

    # Set the frame IDs
    world_frame_id = 'map'
    camera_frame_id = 'body'

    # Create a TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a TF broadcaster
    # tf_broadcaster = tf2_ros.TransformBroadcaster()
    transform_pub = rospy.Publisher('/body', TransformStamped, queue_size=10)


    # Set the rate at which to publish the transform
    rate = rospy.Rate(30.0)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get the transform from the buffer
            transform = tf_buffer.lookup_transform(world_frame_id, camera_frame_id, rospy.Time(0))

            # Create a TransformStamped message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = transform.header.frame_id
            transform_msg.child_frame_id = camera_frame_id
            transform_msg.transform = transform.transform

            # Publish the transform
            transform_pub.publish(transform_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Handle any exceptions that might occur while looking up the transform
            rospy.logwarn('Exception occurred while looking up transform: {}'.format(str(e)))

        # Sleep for the remaining time until the next loop iteration
        rate.sleep()
