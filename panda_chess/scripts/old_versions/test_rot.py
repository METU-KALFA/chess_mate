import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

rospy.init_node('transform_listener')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)


try:
    transform = tf_buffer.lookup_transform('camera_link', 'panda_link0', rospy.Time(0), rospy.Duration(1.0))
    #transform_stamped = tf_buffer.lookup_transform("panda_link0", camer, rospy.Time(0), rospy.Duration(1.0))
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.logwarn("Transform not available!")
    exit(1)

translation = transform.transform.translation
rotation = transform.transform.rotation

#euler = tf2_ros.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])

print ("rotation", rotation)
print ("translation", translation)