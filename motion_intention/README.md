ROS Package for handling the motion_intention (MInt) node. Very WIP, expect constant changes at the moment.

motion_intention_node.cpp:
    Main node for this package. 
    It subscribes to the topic "ee_pose" and expects geometry_msgs/PoseStamped messages.
    Publishes geometry_msgs/PoseStamped messages to topic "ee_pose_eq" to generate end effector equilibrium poses that can be read in by an admittance controller.
     
test_pub.py:
    Test/debug node for generating geometry_msgs/PoseStamped messages to the topic "ee_pose" (end effector pose).

Dependices:
    You'll need to place libtorch into the lib folder. The path should look like "motion_intention/lib/libtorch". You will also need Eigen (https://eigen.tuxfamily.org/index.php?title=Main_Page) and nlohmann's JSON package (https://github.com/nlohmann/json) headers placed into the include directory.

