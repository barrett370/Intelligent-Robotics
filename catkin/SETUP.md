# Setting up catkin & ROS to work with Python3.7

- Make sure you have ROS-Melodic & Catkin installed, see http://wiki.ros.org/melodic/Installation/Ubuntu

- With these installed either add to your .bashrc/.zshrc or manually run \
`source ./catkin/devel/source.(zsh|bash|sh)`
- from a terminal where this has been run, launch pycharm. The rosdeps libraries should be indexed automatically 
- Run:
`pipenv update`
  to install all other python3 dependencies
  
## Add ros to PYTHONPATH

1. Go to `File | Settings | Project: Intelligent-Robotics | Project Interpreter`
2. Select `Show All...` on the Interpreter dropdown
3. Select `Show paths for the selected interpreter`
4. Add `/opt/ros/melodic/lib/python2.7/dist-packages`


 - This should now all work... let me know if there are any issues