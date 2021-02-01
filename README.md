# AirSim Collective Algorithm Testing
This repository is a fork from Microsoft's AirSim platform. It is currently being optimized for obstacle avoidance training models for drones and vehicles, as well as experimentation with Capsule Convolutional Neural Networks (CCNN). The end of goal of this project is to develop a platform to easily test swarm-based, collective sampling state estimation networks and communications patterns.

## Objectives
1. Design a easy-to-use testing harness for verification of state-estimation, collective agent networks.
2. Design, test and implement a software-in-the-loop training process for obstacle avoidance models based upon image and distance data (stereo cameras and lidar/radar data)
3. Research Capsule Convultional Neural Networks for use in obstacle avoidance and adverserial agent detection.
4. Design and implement a distributed, Deep reinforcement learning platform for drones utilizing either Microsoft Azure or AWS for training of complex control modueles and SLAM components.

## How to Get It

[![Build Status](https://travis-ci.org/Microsoft/AirSim.svg?branch=master)](https://travis-ci.org/Microsoft/AirSim)

### Windows
* [Download binaries](https://github.com/Microsoft/AirSim/releases)
* [Build it](https://microsoft.github.io/AirSim/build_windows)

### Linux
* [Download binaries](https://github.com/Microsoft/AirSim/releases)
* [Build it](https://microsoft.github.io/AirSim/build_linux)

### macOS
* [Build it](https://microsoft.github.io/AirSim/build_linux)

For more details, see the [use precompiled binaries](docs/use_precompiled.md) document. 

## How to Use It

### Documentation

*This is the basic documentation*
View our [detailed documentation](https://microsoft.github.io/AirSim/) on all aspects of AirSim.
*A more detailed documentation of changes made and how to utilize this platform will be written during
and upon completion of this project*

### Gathering training data

There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button in the lower right corner. This will start writing pose and images for each frame. The data logging code is pretty simple and you can modify it to your heart's content.

![record screenshot](docs/images/record_data.png)

A better way to generate training data exactly the way you want is by accessing the APIs. This allows you to be in full control of how, what, where and when you want to log data.

### Computer Vision mode

Yet another way to use AirSim is the so-called "Computer Vision" mode. In this mode, you don't have vehicles or physics. You can use the keyboard to move around the scene, or use APIs to position available cameras in any arbitrary pose, and collect images such as depth, disparity, surface normals or object segmentation.

[More details](https://microsoft.github.io/AirSim/image_apis/)

### Weather Effects

Press F10 to see various options available for weather effects. You can also control the weather using [APIs](https://microsoft.github.io/AirSim/apis#weather-apis). Press F1 to see other options available.

![record screenshot](docs/images/weather_menu.png)

## Participate

### Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physica l Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

### Contribute

Please take a look at [open issues](https://github.com/microsoft/airsim/issues) if you are looking for areas to contribute to.

* [More on AirSim design](https://microsoft.github.io/AirSim/design)
* [More on code structure](https://microsoft.github.io/AirSim/code_structure)
* [Contribution Guidelines](CONTRIBUTING.md)

### Who is Using AirSim?

We are maintaining a [list](https://microsoft.github.io/AirSim/who_is_using) of a few projects, people and groups that we are aware of. If you would like to be featured in this list please [make a request here](https://github.com/microsoft/airsim/issues).

## Contact

Join our [GitHub Discussions group](https://github.com/microsoft/AirSim/discussions) to stay up to date or ask any questions.

We also have an AirSim group on [Facebook](https://www.facebook.com/groups/1225832467530667/). 


## What's New

- [Python wrapper for Open AI gym interfaces.](https://github.com/microsoft/AirSim/pull/3215)
- [Python wrapper for Event camera simulation](https://github.com/microsoft/AirSim/pull/3202)
- [Voxel grid construction](https://github.com/microsoft/AirSim/pull/3209)
- [Programmable camera distortion](https://github.com/microsoft/AirSim/pull/3039)
- [Wind simulation](https://github.com/microsoft/AirSim/pull/2867)
- [Azure development environment with documentation](https://github.com/microsoft/AirSim/pull/2816)
- ROS wrapper for [multirotor](https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md) and [car](https://github.com/microsoft/AirSim/pull/2743).

For complete list of changes, view our [Changelog](docs/CHANGELOG.md)

## FAQ

If you run into problems, check the [FAQ](https://microsoft.github.io/AirSim/faq) and feel free to post issues in the  [AirSim](https://github.com/Microsoft/AirSim/issues) repository.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.


## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.



## How to Get It

[![Build Status](https://travis-ci.org/Microsoft/AirSim.svg?branch=master)](https://travis-ci.org/Microsoft/AirSim)

### Windows
* [Download binaries](https://github.com/Microsoft/AirSim/releases)
* [Build it](https://microsoft.github.io/AirSim/build_windows)

### Linux
* [Download binaries](https://github.com/Microsoft/AirSim/releases)
* [Build it](https://microsoft.github.io/AirSim/build_linux)

### macOS
* [Build it](https://microsoft.github.io/AirSim/build_linux)

For more details, see the [use precompiled binaries](docs/use_precompiled.md) document. 

## How to Use It

### Documentation

*This is the basic documentation*
View our [detailed documentation](https://microsoft.github.io/AirSim/) on all aspects of AirSim.
*A more detailed documentation of changes made and how to utilize this platform will be written during
and upon completion of this project*

### Gathering training data

There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button in the lower right corner. This will start writing pose and images for each frame. The data logging code is pretty simple and you can modify it to your heart's content.

![record screenshot](docs/images/record_data.png)

A better way to generate training data exactly the way you want is by accessing the APIs. This allows you to be in full control of how, what, where and when you want to log data.

### Computer Vision mode

Yet another way to use AirSim is the so-called "Computer Vision" mode. In this mode, you don't have vehicles or physics. You can use the keyboard to move around the scene, or use APIs to position available cameras in any arbitrary pose, and collect images such as depth, disparity, surface normals or object segmentation.

[More details](https://microsoft.github.io/AirSim/image_apis/)

### Weather Effects

Press F10 to see various options available for weather effects. You can also control the weather using [APIs](https://microsoft.github.io/AirSim/apis#weather-apis). Press F1 to see other options available.

![record screenshot](docs/images/weather_menu.png)

## Participate

### Paper

More technical details are available in [AirSim paper (FSR 2017 Conference)](https://arxiv.org/abs/1705.05065). Please cite this as:
```
@inproceedings{airsim2017fsr,
  author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
  title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
  year = {2017},
  booktitle = {Field and Service Robotics},
  eprint = {arXiv:1705.05065},
  url = {https://arxiv.org/abs/1705.05065}
}
```

### Contribute

Please take a look at [open issues](https://github.com/microsoft/airsim/issues) if you are looking for areas to contribute to.

* [More on AirSim design](https://microsoft.github.io/AirSim/design)
* [More on code structure](https://microsoft.github.io/AirSim/code_structure)
* [Contribution Guidelines](CONTRIBUTING.md)

### Who is Using AirSim?

We are maintaining a [list](https://microsoft.github.io/AirSim/who_is_using) of a few projects, people and groups that we are aware of. If you would like to be featured in this list please [make a request here](https://github.com/microsoft/airsim/issues).

## Contact

Join the AirSim group on [Facebook](https://www.facebook.com/groups/1225832467530667/) to stay up to date or ask any questions.


## What's New

- [Python wrapper for Open AI gym interfaces.](https://github.com/microsoft/AirSim/pull/3215)
- [Python wrapper for Event camera simulation](https://github.com/microsoft/AirSim/pull/3202)
- [Voxel grid construction](https://github.com/microsoft/AirSim/pull/3209)
- [Programmable camera distortion](https://github.com/microsoft/AirSim/pull/3039)
- [Wind simulation](https://github.com/microsoft/AirSim/pull/2867)
- [Azure development environment with documentation](https://github.com/microsoft/AirSim/pull/2816)
- ROS wrapper for [multirotor](https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md) and [car](https://github.com/microsoft/AirSim/pull/2743).

For complete list of changes, view our [Changelog](docs/CHANGELOG.md)

## FAQ

If you run into problems, check the [FAQ](https://microsoft.github.io/AirSim/faq) and feel free to post issues in the  [AirSim](https://github.com/Microsoft/AirSim/issues) repository.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.


## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.


>>>>>>> 0fce17884cee98d733ce8ede54399b20d879da70
>>>>>>> Fixed README.md
