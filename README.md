# Evaluation of VIO/VSLAM/VO algorithms

## Algorithms Support
- VINS-MONO: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- S-MSCKF: https://github.com/KumarRobotics/msckf_vio
- ORB-SLAM: https://github.com/raulmur/ORB_SLAM2
- SVO2: https://github.com/uzh-rpg/rpg_svo_example
- our MSCKF: Upcoming

## Dataset Support
- EuRoC vio dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- TUM vio dataset(Upcoming): https://vision.in.tum.de/data/datasets/visual-inertial-dataset 

## How to build package and run on dataset?
Follow the directions of build.md inside each fold.
- [Build&Run VINS-MONO](./vins/build.md)
- [Build&Run S-MSCKF](./smsckf/build.md)
- [Build&Run ORB-SLAM](./orbslam/build.md)
- [Build&Run SVO2](./svo2/build.md)

Note: raw versions of VINS-MONO and S-MSCKF are run on ROS, while others run without ROS. So we download two format(.zip, .bag) of EuRoC dataset for convenience.

Strictly speaking, it's unfair for package who run with ROS bag. Since if the processing frequency is less than image publish frequency, it will lose frame data. In zip file, all data will be processed.

## How to evaluate?
We calculate ATE (Absolute Trajectory Error) between estimate trajectory to the groundtruth. RPE (Relative Pose Error) soon.

After build and run, running result will be stored in 'results' folder. Use evaluate_euroc.py to evaluated.
```
python evaluate_euroc.py <vio_result_dir>
```

## How to compare?
You can compare two or more algorithms' results.

```
# Compare two
python compare_evaluate.py alg1_name,alg2_name alg1_result_dir alg2_result_dir
# Compare three
python compare_evaluate.py alg1_name,alg2_name,alg3_name alg1_result_dir alg2_result_dir alg3_result_dir
```

## Results

### Algorithms details
|algorithms| sensor | description |modification|
|--|--|--|--|
|vins|Mono+IMU|Optimization-based VIO, sliding window|Unable loop closure|
|smsckf|Stereo+IMU|Filter-based VIO, Multi-State Constraint Kalman Filter ||
|orbslam|Stereo|Feature-based VSLAM|Unable loop closure|
|svo|Stereo|Semi-direct VO, depth filter||
|ours|Stereo+IMU|Filter-based VIO||

### ATE on EuRoC
|dataset         |   vins  |  smsckf  |orbslam  |   svo2  |   ours |
|----------------|---------|----------|---------|---------|--------|
|sensor          | Mono+IMU|Stereo+IMU|Stereo   |Stereo   |Stereo+IMU|
|MH_01_easy      |0.156025 |   x      |**0.037896** |0.111732 |0.185233|
|MH_02_easy      |0.178418 | 0.152133 |**0.044086** |    x    |0.116650|
|MH_03_medium    |0.194874 | 0.289593 |**0.040688** |0.360784 |0.229765|
|MH_04_difficult |0.346300 | 0.210353 |**0.088795** |2.891935 |0.261902|
|MH_05_difficult |0.302346 | 0.293128 |**0.067401** |1.199866 |0.281594|
|V1_01_easy      |0.088925 | 0.070955 |0.087481 |0.061025 |**0.060067**|
|V1_02_medium    |0.110438 | 0.126732 |0.079843 |0.081536 |**0.071677**|
|V1_03_difficult |0.187195 | 0.203363 |0.284315 |0.248401 |**0.163459**|
|V2_01_easy      |0.086263 | 0.065962 |0.077287 |0.076514 |**0.056529**|
|V2_02_medium    |0.157444 | 0.175961 |0.117782 |0.204471 |**0.097642**|
|V2_03_difficult |**0.277569** |   x      |  x      |   x     | x      |

### Average frame cost on EuRoC(ms)
Details:
- vins: cost of processImage() include back-end estimator, exclude front-end, imu pre-integration.
- smsckf: whole back-end estimator cost.
- orbslam: whole image cost include front-end and tracker thread, exclude mapping thread.
- svo2: whole image cost include include tracker thread, exclude mapping thread.
- ours: same as smsckf

|dataset         |   vins  |  smsckf  |orbslam  |   svo2  |   ours |
|----------------|---------|----------|---------|---------|--------|
|MH_01_easy      |44.0|2.2|62.4| 4.2|11.3|
|MH_02_easy      |43.7|3.4|61.0| 9.4|11.8|
|MH_03_medium    |45.3|3.6|59.4|10.6|12.4|
|MH_04_difficult |40.4|3.6|51.6|10.9|11.1|
|MH_05_difficult |40.8|3.8|50.1|10.3|11.5|
|V1_01_easy      |48.9|3.4|50.4|10.3|12.0|
|V1_02_medium    |33.8|2.9|50.0| 8.5|12.3|
|V1_03_difficult |24.2|2.5|49.4|10.0|12.3|
|V2_01_easy      |43.7|3.2|50.1| 9.2|11.9|
|V2_02_medium    |32.6|2.9|53.8|10.1|13.1|
|V2_03_difficult |20.8|1.6|51.1| 9.2| 9.9|
|Average cost    |38.0|3.0|53.6| 9.3|11.9|
|Average FPS     |26.3|333.3|18.7|107.5|84.0|

### Trajectory on EuRoC
|dataset         |  vins  |  smsckf  |orbslam  |  svo2  | ours |
|----------------|---------|----------|---------|---------|--------|
|MH_01_easy      |![a](./results/MH_01_easy_vins.png     ) |![a](./results/MH_01_easy_smsckf.png     ) |![a](./results/MH_01_easy_orbslam.png     ) |![a](./results/MH_01_easy_svo2.png     ) |![a](./results/MH_01_easy_msckf.png     ) |
|MH_02_easy      |![a](./results/MH_02_easy_vins.png     ) |![a](./results/MH_02_easy_smsckf.png     ) |![a](./results/MH_02_easy_orbslam.png     ) |![a](./results/MH_02_easy_svo2.png     ) |![a](./results/MH_02_easy_msckf.png     ) |
|MH_03_medium    |![a](./results/MH_03_medium_vins.png   ) |![a](./results/MH_03_medium_smsckf.png   ) |![a](./results/MH_03_medium_orbslam.png   ) |![a](./results/MH_03_medium_svo2.png   ) |![a](./results/MH_03_medium_msckf.png   ) |
|MH_04_difficult |![a](./results/MH_04_difficult_vins.png) |![a](./results/MH_04_difficult_smsckf.png) |![a](./results/MH_04_difficult_orbslam.png) |![a](./results/MH_04_difficult_svo2.png) |![a](./results/MH_04_difficult_msckf.png) |
|MH_05_difficult |![a](./results/MH_05_difficult_vins.png) |![a](./results/MH_05_difficult_smsckf.png) |![a](./results/MH_05_difficult_orbslam.png) |![a](./results/MH_05_difficult_svo2.png) |![a](./results/MH_05_difficult_msckf.png) |
|V1_01_easy      |![a](./results/V1_01_easy_vins.png     ) |![a](./results/V1_01_easy_smsckf.png     ) |![a](./results/V1_01_easy_orbslam.png     ) |![a](./results/V1_01_easy_svo2.png     ) |![a](./results/V1_01_easy_msckf.png     ) |
|V1_02_medium    |![a](./results/V1_02_medium_vins.png   ) |![a](./results/V1_02_medium_smsckf.png   ) |![a](./results/V1_02_medium_orbslam.png   ) |![a](./results/V1_02_medium_svo2.png   ) |![a](./results/V1_02_medium_msckf.png   ) |
|V1_03_difficult |![a](./results/V1_03_difficult_vins.png) |![a](./results/V1_03_difficult_smsckf.png) |![a](./results/V1_03_difficult_orbslam.png) |![a](./results/V1_03_difficult_svo2.png) |![a](./results/V1_03_difficult_msckf.png) |
|V2_01_easy      |![a](./results/V2_01_easy_vins.png     ) |![a](./results/V2_01_easy_smsckf.png     ) |![a](./results/V2_01_easy_orbslam.png     ) |![a](./results/V2_01_easy_svo2.png     ) |![a](./results/V2_01_easy_msckf.png     ) |
|V2_02_medium    |![a](./results/V2_02_medium_vins.png   ) |![a](./results/V2_02_medium_smsckf.png   ) |![a](./results/V2_02_medium_orbslam.png   ) |![a](./results/V2_02_medium_svo2.png   ) |![a](./results/V2_02_medium_msckf.png   ) |
|V2_03_difficult |![a](./results/V2_03_difficult_vins.png) |![a](./results/V2_03_difficult_smsckf.png) |![a](./results/V2_03_difficult_orbslam.png) |![a](./results/V2_03_difficult_svo2.png) |![a](./results/V2_03_difficult_msckf.png) |
