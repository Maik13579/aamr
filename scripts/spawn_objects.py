import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def get_cylinder_sdf(radius, length):
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name="graspable_cylinder">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


if __name__ == '__main__':
    import time
    time.sleep(15)
    rospy.init_node('object_spawner')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    pose = Pose()
    pose.position.x = -3.8
    pose.position.y = 3.8
    pose.position.z = 2.0
    pose.orientation.w = 1.0
    spawn_model('graspable_cylinder', get_cylinder_sdf(0.025, 0.1), '', pose, '')