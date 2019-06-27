# Test SVO2 on EuRoC dataset

## How to build SVO2?
SVO2.0 code is not open, only binaries available in http://rpg.ifi.uzh.ch/svo2.html.
There is a sample code on ROS here (https://github.com/uzh-rpg/rpg_svo_example).

We build a ROS-free demo code based on rpg_svo_example. The code is release https://github.com/symao/svo2_noros.

Note: We use the SVO2.0 binaries in Ubuntu16.04 + opencv3.3.1. The version must be matched strictly.

```
## download code
git clone https://github.com/symao/svo2_noros

## checkout to modify version which close viewer and loop closing
cd svo2_noros
git checkout master

## build
mkdir build
cd build; cmake ..; make -j
```

## How to run SVO2 on EuRoC datasets?
We choose demo_stereo_euroc in SVO2 for test. If you want to test mono version, use the demo_mono_euroc.
stereo_euroc run on EuRoC datasets in ASL Dataset Format(.zip). So this py script doesn't support ROS bag. If you want to use rosbag, try to use the ROS version.

1. Prepare
 - Download EuRoC dataset on https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets. NOTE: Download datasets in **ASL Dataset Format(.zip)** rather than rosbag.
 - Unzip all dataset. Copy 'unzip_dataset.py' in folder which contains all zip files. Run 'python unzip_dataset.py'
2. Batch Run
 - Modify param/euroc_stereo_imu.yaml, set the calib_file path for your own.
 > calib_file: /home/symao/workspace/svo2_noros/calib/euroc_stereo.yaml
 - Modify 'orb_dir','whole_dir','res_dir' in run_euroc.py
  - orb_dir: your SVO2.0 code path
  - whole_dir: the path which contain all unzip datasets.
  - res_dir: the path to save run result.
 - run 'python run_euroc.py'

