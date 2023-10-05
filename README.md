# Variable Impedance Controller with Online Motion Intent Prediction
ROS metapackage repo for implimenting variable impedance control with motion intention prediction experiments. 
It's intented to be used with the iiwa_ros repo and private admittance_control repo.

The current metapackage has the following components:
| Components | Description |
| ---- | --- |
| Motion Intention| ROS Package that handles online motion intention forecasting using Autoregressive Neural Networks |
| Protocol Controller | ROS Package that implements the experiment protocol using state machine library SMACH |

# More about Motion Intention



# More about Protocol Controller
The Protocol Controller uses the Python library [SMACH](http://wiki.ros.org/smach), a task-level architecture for rapidly creating complex robot behavior, to implement a state machine with the following states:
- [Initial State](#More-About-Initial-State)
- [Origin Holding](#More-About-Origin-Holding)
- [Trial Set](#More-About-Trial-Set)
  - [Init](#More-About-Init-State)
  - [Calibration](#More-About-Calibration)
  - [Fit Trial Block](#More-About-Fit-Trial-Block)
 
![State Machine FLow](./docs/source/images/smach_viewer.png)
 
# More About Initial State
The Initial State initializes the global variables that are used consistently throughout the experiment. The output keys for the state are Default Stiffness and Damping Parameters, senstitivity constant, shifting constant and tuning parameters. After the variables are successfully initialized, the state exits with the outcome "Intialized" to the next state Origin Holding.

# More About Origin Holding
The Origin Holding helps the robot to move to a pre-defined origin postion defined by the particular joint angles at a stable and safe pace. The state allows us to resort to the origin pose while initiating a trial or when anything goes awry during the experiments. 
# More About Trial Set
The Trial Set is a hierarchical state machine with children states Init, Calibration and Fit Trial Block. It first executes the Init State that initializes trial related variables and prepares for ensuing trial blocks. Subseequently, we calibrate our control parameters to adapt to each user through a set of trials, following which we execute a set of fitting trials using our Motion Intent Network to forecast user intent during trials and other contemporary fitting methods like Linear Fitting and Gradient Weighted Algebraic Fitting to gauge its performance. 

