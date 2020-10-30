CONTROLLER_WORKING_DIRECTORY=./devel/lib/controller_pkg
mkdir -p $CONTROLLER_WORKING_DIRECTORY
ln -sf `pwd`/doc $CONTROLLER_WORKING_DIRECTORY
roslaunch controller_pkg controller.launch --screen
