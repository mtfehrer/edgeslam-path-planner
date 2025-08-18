To record a bag file, do:
docker-compose up mobile
Then in the container run rosbag record

to play a recording, do:
rosbag play /home/edgeslam/rgbd_dataset_freiburg2_desk.bag /camera/rgb/image_color:=/camera/rgb/image_raw /camera/depth/image:=/camera/depth_registered/image_raw