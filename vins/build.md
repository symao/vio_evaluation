# Test VINS-MONO on EuRoC dataset

## How to build VINS-MONO?
To build VINS-MONO, you need install ROS first. See (http://www.ros.org/install/) for details.
We modify code to save vio trajectory for later evaluation. Note: trajectory logging is in a separated thread, so that estimator thread won't be blocked by logging.
Loop clousure are closed for fair comparison to other VIO algorithms.
```
## download code
cd ~/catkin_ws/src
git clone https://github.com/symao/VINS-Mono

## checkout to modify version which close viewer and loop closing
cd VINS-Mono
git checkout evaluation

## build
cd ~/catkin_ws
catkin_make
```

## How to run VINS-MONO on EuRoC datasets?
1. Download EuRoC dataset on https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. NOTE: Download datasets in **rosbag(.bag)** rather than ASL Dataset Format(*.zip).
2. Batch run. Modify 'bag_dir' to your path which contains all rosbag file. And run 'python run_euroc.py'. It will run all rosbag files in bag_dir.