import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def get_link_pose(link_name, base_frame):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    try:
        transform_stamped = tf_buffer.lookup_transform(base_frame, link_name, rospy.Time(0), rospy.Duration(1.0))
        return transform_stamped.transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn(f"Failed to get transform for {link_name} in {base_frame}")
        return None

if __name__ == "__main__":
    rospy.init_node("link_pose_listener")
    link_name = "camera_link"  # Replace with the name of the link you want to get the pose of
    base_frame = "panda_link0"      # Replace with the name of the reference frame you want to express the pose in

    link_pose = get_link_pose(link_name, base_frame)
    if link_pose:
        print("Link Pose:")
        print("Position (XYZ):", link_pose.translation.x, link_pose.translation.y, link_pose.translation.z)
        print("Orientation (Quaternion):", link_pose.rotation.x, link_pose.rotation.y, link_pose.rotation.z, link_pose.rotation.w)