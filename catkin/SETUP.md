# Setting up catkin & ROS to work with Python3.7

- Make sure you have ROS-Melodic & Catkin installed, see http://wiki.ros.org/melodic/Installation/Ubuntu

- With these installed either add to your .bashrc/.zshrc or manually run 
```bash
source ./catkin/devel/source.(zsh|bash|sh)
```
- from a terminal where this has been run, launch pycharm. The rosdeps libraries should be indexed automatically 
- Run:
```bash 
pipenv update  
```
  to install all other python3 dependencies
  
 - This should now all work... let me know if there are any issues