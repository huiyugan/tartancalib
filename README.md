# TartanCalib

[![ROS1 Ubuntu 20.04](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml/badge.svg)](https://github.com/ethz-asl/kalibr/actions/workflows/docker_2004_build.yaml)

## Introduction
TartanCalib contributes state-of-the-art calibration for wide-angle lenses, by implementing an iterative calibration pipeline and adaptive subPixel refinement of AprilTags. The code for TartanCalib is built upon the [Kalibr](https://github.com/ethz-asl/kalibr) toolbox, is easy to use, and robust. If you are interested in helping us extend the features of TartanCalib, please reach out to Bart ([email](bduister@andrew.cmu.edu)). 

For more information, visit our project page [here](https://tartancalib.com).

## Installation
### Docker
To use the Dockerfiles provided in this repository, ensure that you have [Docker](https://docs.docker.com/get-docker/) installed on your system.

1. Clone and build docker image
        
        git clone https://github.com/castacks/tartancalib
        cd tartancalib
        docker build -t tartancalib -f Dockerfile_ros1_20_04 .

2. Mount data folder and share xhost within container for GUI. For more information, read the [ROS wiki](http://wiki.ros.org/docker/Tutorials/GUI) on Docker.
        
        FOLDER=/path/to/your/data/on/host
        xhost +local:root
        docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
            -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            -v "$FOLDER:/data" tartancalib

3. When in the Docker container, follow the [Usage](#usage) section of this readme to run calibration commands.

### Local Installation
TartanCalib and Kalibr are integrated within ROS. Ensure that you have ROS installed within your system.
- Ubuntu 16.04 ROS 1 [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Ubuntu 18.04 ROS 1 [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Ubuntu 20.04 ROS 1 [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

1. Get dependencies

Ubuntu 16

        sudo apt-get update && apt-get install -y \
        git wget autoconf automake \
        python2.7-dev python-pip python-scipy python-matplotlib \
        ipython python-wxgtk3.0 python-tk python-igraph \
        libeigen3-dev libboost-all-dev libsuitesparse-dev \
        doxygen \
        libopencv-dev \
        libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
        python-catkin-tools     

Ubuntu 18

        sudo apt-get update && apt-get install -y \
        git wget autoconf automake nano \
        python3-dev python-pip python-scipy python-matplotlib \
        ipython python-wxgtk4.0 python-tk python-igraph \
        libeigen3-dev libboost-all-dev libsuitesparse-dev \
        doxygen \
        libopencv-dev \
        libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
        python-catkin-tools


Ubuntu 20

        sudo apt-get update && apt-get install -y \
        git wget autoconf automake nano \
        python3-dev python3-pip python3-scipy python3-matplotlib \
        ipython3 python3-wxgtk4.0 python3-tk python3-igraph \
        libeigen3-dev libboost-all-dev libsuitesparse-dev \
        doxygen \
        libopencv-dev \
        libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
        python3-catkin-tools python3-osrf-pycommon

2. Clone and build repository

        mkdir ~/tartan_ws && cd ~/tartan_ws
        mkdir src && cd src
        git clone https://github.com/castacks/tartancalib
        cd ..
        export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
        source /opt/ros/$ROS1_DISTRO/setup.bash
        catkin init
        catkin config --extend /opt/ros/$ROS1_DISTRO
        catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

3. Source catkin workspace
        
        cd ~/tartan_ws && source devel/setup.bash

4. Follow [Usage](#usage) section of this ReadMe to run calibration commands.

## Usage
1. Create rosbag dataset

This can be done through a multitude of different ways. If you have a physical setup linked to ROS, directly recording the sensor stream to a [rosbag](http://wiki.ros.org/rosbag) would be  easiest. There are also methods of cutting common video formats into rosbags. The rosbag [api](http://wiki.ros.org/rosbag/Code%20API) is well documented and contains examples that would potentially be helpful if you are new to working with bag files. 

2. Define calibration board through YAML files

Kalibr supports three different calibration targets with different parameters associated to each target. Kalibr's [wiki](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) has more information on calibration targets in YAML. An example of an aprilgrid YAML file is provided below:

        #example for aprilgrid
        target_type: 'aprilgrid' #gridtype
        tagCols: 10                 #number of apriltags
        tagRows: 7                  #number of apriltags
        tagSize: 0.025              #size of apriltag, edge to edge [m]
        tagSpacing: 0.3             #ratio of space between tags to tagSize
                                    #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]

The method should also work with checkerboards but would require some adaptation. In general grids of AprilTags are most robust.

3. Run calibration

This is sample code for running calibration with certain parameters changed. Please refer to the table below for the full range of configurable parameters.

In a terminal or bash file:
        
        export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
        source /opt/ros/$ROS1_DISTRO/setup.bash

In a terminal, for example:
        
        cd ~/tartan_ws && source devel/setup.bash
        rosrun kalibr tartan_calibrate \
        --bag /path/to/bagfile.bag \
        --target /path/to/target.yaml \
        --topics /camera_0/image_raw /camera_1/image_raw \
        --min-init-corners-autocomplete 29 \
        --min-tag-size-autocomplete 2 \
        --correction-threshold 10.1 \
        --models omni-radtan omni-radtan \
        --dont-show-report
        --save_dir /path/to/output/ \

Below is a table of configurable parameters that you may use to customize TartanCalib to your needs:

| Parameter                     | Description                                                                           |
| ----------------------------- | -----------                                                                           |
| bag                           | Path to calibration bagfile.                                                          |
| target                        | Path to target yaml files.                                                            |
| topics                        | Topics within bag file to calibrate. For multi-cam calibration, specify topics you'd like to calibrate. For example, if you have three cameras with topic names `/camera_n/image_raw`, use `--topics camera_0/image_raw camera_1/image_raw camera_2/image_raw`    |
| models                        | Choose from four possible projections: pinhole, homography, cornerpredictor and none. Cornerpredictor is the 
| fovs                          | If using pinhole projection mode, this parameter represents FOV of pinhole. This argument accepts multiple pinholes corresponding to each topic. For example, if your bag contains two topics, `/camera_0/image_raw`, and `/camera_1/image_raw`, to generate three pinholes of fovs 30, 60, 90 for `camera_0` and one pinhole of fov 90 for `camera_1`, use `--fovs 30 60 90 --fovs 90`. Note that the number of arguments for fovs must correspond with poses and resolutions.           |
| poses                         | If using pinhole projection mode, this parameter represents pose of pinhole. As with fovs, this argument can be configured for multiple projections.          |
| resolutions                   | If using pinhole projection mode, this parameter represents resolution of pinhole. As with fovs, this argument can be configured for multiple projections.   |
| projections                   | Choose from four possible projections: `pinhole`, `homography`, `cornerpredictor` and `none`. `Cornerpredictor` is the autocomplete method described within the paper. |
| debug-modes                   | Choose from 7 possible debug modes for additional debug images: `pointcloudprojection`, `pinholeprojection`, `originalprojection`, `targetpointcloud`, `individualprojections`, `reprojectionpoints` and `none`. |
| min-init-corners-autocomplete | The algorithm requires this many corners minimally for autocomplete. Smaller values might allow the algoirthm to run better, but the pose of the board might be too uncertain for accurate results.            |
| min-tag-size-autocomplete     | Minimum number of pixels a tag needs to be before being autocompleted. This ensures really small tags are excluded instead of being detected poorly and affecting calibration.                               |
| correction-threshold          | Number of pixel offset between reprojection and detection allowed.                    |
| min-resize-window-size        | Minimum window size allowed during dynamic sizing.                                    |
| max-resize-window-size        | Maximum window size allowed during dynamic sizing.                                    |
| symmetry-refinement           | Boolean option for if experimental feature symmetry refinement is allowed.            |
| symmetry-edge-threshold       | Checks if detections are too close to border of pixels.                               |
| log_dest                      | Save directory of logs.                                                               |
| outputMatlab                  | Outputs file in Matlab format for use in BabelCalib or similar calibration toolboxes. |
| save_dir                      | Save directory for logging data.                                                      |

Output will be found in the directory you've specified in save_dir.

4. Output
TartanCalib runs in two iterations. The output file using enhancements from TartanCalib may be found in `log1-camchain.yaml`. The .yaml file will contain distortion coefficients for the distortion model used, as well as intrinsics for the camera being calibrated. For multi-camera calibration, the .yaml file will also include extrinsics between different cameras.

For more information on the output, refer to Kalibr's [wiki](https://github.com/ethz-asl/kalibr/wiki/yaml-formats).

## Authors
* Bardienus P. Duisterhof ([email](bduister@andrew.cmu.edu))
* Yaoyu Hu
* Si Heng Teng ([email](sihengt@andrew.cmu.edu))
* Michael Kaess
* Sebastian Scherer

## References
The calibration approaches used in TartanCalib are documented in the following paper. Please cite this paper, and the appropriate papers on [Kalibr's](https://github.com/ethz-asl/kalibr/) repository when using this toolbox or parts of it in an academic publication.

        @article{duisterhof2022tartancalib,
          title={TartanCalib: Iterative Wide-Angle Lens Calibration using Adaptive SubPixel Refinement of AprilTags},
          author={Duisterhof, Bardienus P and Hu, Yaoyu and Teng, Si Heng and Kaess, Michael and Scherer, Sebastian},
          journal={arXiv preprint arXiv:2210.02511},
          year={2022}
        }

## Acknowledgments
This work is built upon the [Kalibr](https://github.com/ethz-asl/kalibr) toolbox.

## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

4. Neither the name of the Autonomous Systems Lab and Skybotix AG and Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB, SKYBOTIX AG AND CARNEGIE MELLON UNIVERSITY ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG OR CARNEGIE MELLON UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
