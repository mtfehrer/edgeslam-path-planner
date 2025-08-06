For simplicity, this repository will only support the following:
A monocular setup running a bag file in a centralized environment.

There will be no support for the following:
ZED camera, NVIDIA GPU, Distributed environment.

The visualizer, Pyvista isn't run inside a docker container because there are too many dependencies to install. Needs same GPU drivers installed on the host.

planner container uses output directory because atomic rename screws with docker's file bind mount