# jackal_aptag
NOTE: apriltag and apriltag_ros should be in the src folder:
This repo has 2 apriltag with id 1 and 3 with their model.config and model.sdf files in the models folder. 
you can copy the models folder and empty_world file to your package. If you want to add other tags you first got to get a .dae file of the box with the apriltag on it and modify copy the apriltag_1 folder as a template and chage accroding to the new files names. I am goingt to be adding a script that automaticaly generates the models.

To use this package:
to launch the world with two apriltags facing each other you have to run:
roslaunch apriltag_gazebo jack_trial.launch

and make sure that you GAZEBO_PATH_:includes the path to your models file.
