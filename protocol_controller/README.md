The current visualizer execution is heavily dependent on the user's system. A more modular and streamlined launch will be developed soon. 
Until then, the following steps ought to be followed to successfully execute the state machine WITH THE VISUALIZER:
1. Ensure the package is built:
'''
cd /root/catkin_ws/
catkin_make
source devel/setup.bash
'''

3. Set the display environment for your docker in YOUR HOST MACHINE (Pre-req: Install Vcxsrv or any other X Server) : 
'''
docker exec [CONTAINER_ID_OR_NAME] /bin/bash -c "echo 'export DISPLAY=host.docker.internal:0.0' >> ~/.bashrc"
docker exec -it [CONTAINER_ID_OR_NAME] /bin/bash
'''

3. Source the workspace in every terminal and launch the protocol controller if needed
'''
source /root/catkin_ws/devel/setup.bash
roslaunch protocol_controller standalone.launch
'''

4. Run smach viewer (The X Server should be open beforehand)
'''
rosrun smach_viewer smach_viewer.py
'''
