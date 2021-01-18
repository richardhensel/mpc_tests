# MPC Tests

Some experiments with Model Predictive Control of a ground vehicle. 

Built with ROS2 Dashing. 

Visualisation with rviz2.

## Build
Beginning from this directory:
~~~
cd ../
~~~
~~~
colcon build 
~~~

## Run
Beginning from this directory:
~~~
cd ../
~~~
~~~
ros2 launch mpc_tests_launch/launch/mpc_tests_launch.py
~~~



## TODO

* move the autogenerate acado code into mpc_controller
* move the mpc controller library into mpc_tests
* Make the mpc controller completely ros independant
* make a new node in a new package to wrap mpc controller 
* learn how to install and link to acado
