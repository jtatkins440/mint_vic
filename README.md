# Variable Impedance Controller with Online Motion Intent Prediction
ROS metapackage repo for implimenting variable impedance control with motion intention prediction experiments. 
It's intented to be used with the iiwa_ros repo and private admittance_control repo.

The current metapackage has the following components:
| Components | Description |
| ---- | --- |
| Motion Intention| ROS Package that handles online motion intention forecasting using Autoregressive Neural Networks |
| Protocol Controller | ROS Package that implements the experiment protocol using state machine library SMACH |
