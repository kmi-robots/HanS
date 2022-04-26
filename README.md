# HanS
Visual sensemaking repo for the Health &amp; Safety robot inspector (HanS)

## Dependencies

* ROS distro: galactic 
* [ROS2](https://docs.ros.org/en/galactic/Installation.html) 
  [Instructions for setting up a ROS2 workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html). 
  Then, this repo can be cloned under the ```src``` folder. 
  
* Bounding box detection via OpenCV. 
  Object classification via USB, through a Coral Edge TPU.
  Setting up the Edge TPU accelerator and Pycoral library: [https://coral.ai/docs/accelerator/get-started](https://coral.ai/docs/accelerator/get-started)

* [Open3D Python library](http://www.open3d.org/docs/0.12.0/introduction.html) v.0.12 for 3D data processing

* ```apt install ros-galactic-sensor-msgs-py```


## Database setup
Instructions in ```./postgresql``` folder. 
Create tables first through the queries in the ```create_queries.sql``` file.
Apply triggers to queries as explained in the ```trigger_functions.md``` file.

