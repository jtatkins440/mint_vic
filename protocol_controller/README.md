The current visualizer execution is heavily dependent on the user's system. A more modular and streamlined launch will be developed soon. 
Until then, the following steps ought to be followed to successfully execute the state machine WITH THE VISUALIZER:

**Ensure that the package is built**

```bash
cd /root/catkin_ws/
catkin_make
source devel/setup.bash
```

**Set Display Environment in your HOST MACHINE**
```bash
docker exec [CONTAINER_ID_OR_NAME] /bin/bash -c "echo 'export DISPLAY=host.docker.internal:0.0' >> ~/.bashrc"
docker exec -it [CONTAINER_ID_OR_NAME] /bin/bash
```
**Source the workspace in every terminal and launch the protocol controller**
```bash
source /root/catkin_ws/devel/setup.bash
roslaunch protocol_controller standalone.launch
```

**Run smach viewer**
```bash
rosrun smach_viewer smach_viewer.py
```
