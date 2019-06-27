# Test ORB_SLAM2 on EuRoC dataset

## How to build ORB_SLAM2?
We modify ORB_SLAM2 by unable loop closing to evaluate the VO trajectory, since loop closing will eliminate accumulated error.
We also closing viewer thread to reduce running time on dataset.
Note: If you don't want to close loop closing and viewer thread, It's OK that you download raw ORB_SLAM2 code from https://github.com/raulmur/ORB_SLAM2.
```
## download code
git clone https://github.com/symao/ORB_SLAM2

## checkout to modify version which close viewer and loop closing
cd ORB_SLAM2
git checkout evaluate

## decompress vocabulary file
cd Vocabulary; tar -xzf ORBvoc.txt.tar.gz; cd ..

## build
mkdir build
cd build; cmake ..; make -j
```

## How to run ORB_SLAM2 on EuRoC datasets?
We choose stereo_euroc in ORB_SLAM2 for test. If you want to test mono version, use the mono_euroc.
stereo_euroc run on EuRoC datasets in ASL Dataset Format(.zip). So this py script doesn't support ROS bag. If you want to use rosbag, try to compile the ROS version.

1. Prepare data
 - Download EuRoC dataset on https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. NOTE: Download datasets in **ASL Dataset Format(.zip)** rather than rosbag.
 - Unzip all dataset. Copy 'unzip_dataset.py' in folder which contains all zip files. Run 'python unzip_dataset.py'
2. Batch run
 - Modify 'orb_dir','whole_dir','res_dir' in run_euroc.py
  - orb_dir: your ORB_SLAM2 code path
  - whole_dir: the path which contain all unzip datasets.
  - res_dir: the path to save run result.
 - run 'python run_euroc.py'

