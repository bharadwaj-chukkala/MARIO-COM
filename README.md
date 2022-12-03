# Acme Robotics: Mobile Autonomous Robot for Indoor and Outdoor . Collection Of Medical Waste (MARIO-COM)

## Overview

With the successful design and deployment of a robotic manipulator-based software solution for the automation of assembly line production of automobiles, Acme Robotics, Inc. has been able to maximize the efficiency of the automobile plant. As a part of its 5-year roadmap to further expand its expertise areas and leverage robotic solutions to touch human lives, a mobile robot-based medical waste collection and disposal solution is proposed to efficiently and safely transport medical waste trash bins from different locations inside a hospital to the disposal zone.

Hospital waste transport and handling is a time-consuming, hazardous and infectious process as the staff is exposed to be in contact with medical and bio-hazardous substances. The biomedical waste collected from hospital should not be stored in bins for more than 24 hours and has to be disposed immediately. In order to avoid this we need to ensure regular and safe disposal of waste to help the hospital maintain a clean environment for the patients, staff and visitors. There needs to exist a practical solution, if not one stop, that can take out the risk in the disposal and also be a safe and sustainable way to use technology for the betterment of human lives

The software design and development process shall involve adhering to an amalgamation of Agile Iterative Processes and pair-programming techniques over a course of three weeks. The efficacy of the implementation shall be verified using the test-driven development approach. The driver and navigator roles are exchanged during each phase of the project to effectively develop and complete the backlogs.

## Purpose & Methodology

The Robot will identify the bins in the surrounding environment and dispose them at a designated site. The robot will be deployed from an initial position from which it will traverse through the environment while scanning its nearby surroundings to check if a bin is present in the locality. If the robot recognizes a medical waste bin, it will move towards it, pick it up and dispose the trash in it at the site. After doing that it will rendezvous to the bin position to place the empty bin it it's place and start moving around to find other bins.

This Product will eliminate the usage of manual labour to dispose hazardous waste, thus restricting risks to human lives. Automating the process of disposal also saves the havoc of missing or delaying the disposal. Besides reducing human interaction in the process, it is also way more efficient than a typical human being, thus being an advancement for the better.

The product is a ROBOTIS turtlebot with Open Manipulator X Gripper attached to it. As the turtlebot moves around in the environment, the Manipulator attached to it will pick the bins and dispose the trash. The Product uses perception principles to identify a bin, pick it from it's initial position and place it back. The robot will have a visual sensor to perform the perception aspect of the product. The robot runs on a software with a built in search algorithm to traverse through the environment.

Assumptions:
* The Trigger for the robot to start it's search will be a set time in the night, because hospitals have lesser traffic in the night times.
* The Robot will move towards the closest bin in it's reach, if it encounters multiple bins in its visual frame
* All the trash cans in the hospital scene will be mostly full at the end of the day, so we assume full bins when the robot starts its search
* Once the trash cans are disposed, the robot will place the bin back at its initial position and tag it as empty so that it wont pick it up again until next disposal iteration.

## Personnel:

* **Bharadwaj Chukkala**<br>
UID: 118341705<br>
Bharadwaj Chukkala is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include Machine Learning, Perception and Path Planning.<br>
Contact: bchukkal@umd.edu<br>
LinkedIn: https://www.linkedin.com/in/bharadwaj-chukkala/
* **Adarsh Malapaka**<br>
UID: 118119625<br>
Adarsh Malapaka is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include Control Systems, Robot Kinematics, Path Planning, and ADAS.<br>
Contact: amalapak@terpmail.umd.edu<br>
LinkedIn: https://www.linkedin.com/in/adarsh-malapaka/
* **Kumara Ritvik Oruganti**<br>
UID: 117368963<br>
Kumara Ritvik Oruganti is currently a Master's student in Robotics at the University of Maryland, College Park, MD (Batch of 2023). His interests include developing sustainable, efficient and intelligent robots and embedded systems for real-world problems.<br>
Contact: okritvik@terpmail.umd.edu<br>
Website: https://www.okritvik.com/

## Agile-Iterative Process (AIP) Resources

### Pair Programming

| `Phase` | `Adarsh Malapaka` | `Bharadwaj Chukkala` | `Kumara Ritvik Oruganti` |
| --- | --- | --- | --- |
| 1 | Navigator | Driver | Design Keeper |
| 2 | Driver | Design Keeper | Navigator |
| 3 | Design Keeper | Navigator | Driver |

### Product Backlog:

https://docs.google.com/spreadsheets/d/1XZ4UL9ePxGej_Jj6zL2NBFaGKRe57Hb5tn70BDvRW7s/edit#gid=0

### Sprint Planning Notes

https://docs.google.com/document/d/1JswTIkvGNDT8kqs0M8pbFIWWatNoKk5qQq9jqdAfdCM/edit

### Quad Chart

![quad_chart](INSERT)

![](https://user-images.githubusercontent.com/106445479/202936527-bbe36f9c-a4b2-44f4-b74f-a6e1f2ca8372.png)


## Developer Documentation


### Dependencies
1. [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html): ROS2 Foxy installed on an Ubuntu 20.04 machine is used for this project. The steps to install ROS2 Foxy are mentioned in the above link. 
2. [OpenCV](https://www.opencv-srf.com/p/introduction.html): OpenCV is used to detect the medical waste bins using classical image processing algorithms and can be installed from this [link.](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/)
3. [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/): The Turtlebot3 ROS2 simulation packages are used to simulate the medical waste disposal robot. Upon installing and configuring the ROS2 workspace, the steps to install these packages are mentioned in the above link. 
4. [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator): The ROBOTIS OpenMANIPULATOR-X gripper mounted on the TurtleBot3 robot is used to pick and place the disposal bins. The steps to install and visulaize the robot with this gripper are given in the link above.
5. [NumCpp](https://dpilger26.github.io/NumCpp/doxygen/html/index.html): NumCpp shall be used to make use of NumPy functionalities in the definition of matrices and their associated operations in C++. This can be installed from the above link.

### Environment Setup

Clone the following repo in any directory and place the ```mario_com``` folder in your ROS2 workspace 
```
git clone https://github.com/bharadwaj-chukkala/ENPM808X-Final-Project.git
```


### Installation via Command Line

```
# Code Coverage
sudo apt-get install -y -qq lcov
```

```
# OpenCV install
sudo apt-get install -y build-essential
sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
# Download v4.6.0
curl -sL https://github.com/Itseez/opencv/archive/4.6.0.zip > opencv.zip
unzip opencv.zip
cd opencv-4.6.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
cd ../../
```

```
# Static Code Analysis
sudo apt install cpplint
sudo apt install cppcheck
```

```
# Valgrind
sudo apt install valgrind
sudo apt-get install -y kcachegrind
```

```
# Doxygen
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```

***

### Build, Run and Code Coverage

```
# Clone
git clone https://github.com/bharadwaj-chukkala/ENPM808X-Final-Project.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
# Run
Run tests: ./test/cpp-test
Run the perception module: ./app/shell-app
```

```
# Static Code Analysis
1. cppcheck
bash run_cppcheck.sh
2. cpplint
bash run_cpplint.sh
```

Note: Static Code Analysis Results are stored in `./results`

```
# Valgrind
valgrind --leak-check=full <path of the executable>
valgrind --tool=callgrind  ./app/shell-app
kcachegrind
```

```
# Code Coverage
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
cd coverage
firefox index.html
```

### Doxygen Documentation

```
cd ..
doxygen doxygen.config
doxywizard
```

## License

This project is licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0) and can be found in the `LICENSE` file.


<!-- # Medical Trash Collection Robot

[![Build Status](https://app.travis-ci.com/spauly98/ENPM808X_Midterm_project.svg?branch=phase2)](https://app.travis-ci.com/github/bharadwaj-chukkala/ENPM808X_Midterm_project)
[![Coverage Status](https://coveralls.io/repos/github/spauly98/ENPM808X_Midterm_project/badge.svg?branch=phase2)](https://coveralls.io/github/spauly98/ENPM808X_Midterm_project?branch=phase2)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Overview

### Problem Statement

Hospital waste transport and handling is a time-consuming, hazardous and infectious process as the staff is exposed to be in contact with medical and bio-hazardous substances. The biomedical waste collected from hospital should not be stored in bins for more than 24 hours and has to be disposed immediately. In order to avoid this we need to ensure regular and safe disposal of waste to help the hospital maintain a clean environment for the patients, staff and visitors. There needs to exist a practical solution, if not one stop, that can take out the risk in the disposal and also be a safe and sustainable way to use technology for the betterment of human lives.

### Objective

The Project is to solve the aforementioned problem, we intend to develop a robot that can safely dispose the medical waste in hospitals, from the bins to a designmated disposal site from time to time. The Robot t

### Development Goals

- <------ What are the software developement goals----->
- <------ Product Scalabaility to further acheive ambitious goals------>
- <------ Product Software Fucntionlaity oursourcing --------> 

### Aim of the Product

The Robot will identify the bins in the surrounding environment and dispose them at a designated site. The robot will be deployed from an initial position from which it will traverse through the environment while scanning its nearby surroundings to check if a bin is present in the locality. If the robot recognizes a medical waste bin, it will move towards it, pick it up and dispose the trash in it at the site. After doing that it will rendezvous to the bin position to place the empty bin it it's place and start moving around to find other bins.

This Product will eliminate the usage of manual labour to dispose hazardous waste, thus restricting risks to human lives. Automating the process of disposal also saves the havoc of missing or delaying the disposal. Besides reducing human interaction in the process, it is also way more efficient than a typical human being, thus being an advancement for the better.

- <-------- What is  the product supposed to do------>
- <-------- How is it going to betteer the lives of customers-------->

#### Product Description

The product is a ROBOTIS turtlebot with Open Manipulator X Gripper attached to it. As the turtlebot moves around in the environment, the Manipulator attached to it will pick the bins and dispose the trash. The Product uses perception principles to identify a bin, pick it from it's initial position and place it back. The robot will have a visual sensor to perform the perception aspect of the product. The robot runs on a software with a built in search algorithm to traverse through the environment.

#### Assumptions [Product Working Scenarios]

- The Trigger for the robot to start it's search will be a set time in the night, because hospitals have lesser traffic in the night times.
- The Robot will move towards the closest bin in it's reach, if it encounters multiple bins in its visual frame
- All the trash cans in the hospital scene will be mostly full at the end of the day, so we assume full bins when the robot starts its search
- Once the trash cans are disposed, the robot will place the bin back at its initial position and tag it as empty so that it wont pick it up again until next disposal iteration.

<p align="center"> 
  <img width="500" height="300" src="https://user-images.githubusercontent.com/106445479/202936527-bbe36f9c-a4b2-44f4-b74f-a6e1f2ca8372.png">
</p>
<h4 align="center">Collection Robot picking a Rubik's Cube</h1>

---
## License

MIT License

Copyright (c) 2022 Adarsh Malapaka, Bharadwaj Chukkala, Kumara Ritvik Oruganti

```
DISCLAIMER: Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

### Team Members

| `Adarsh Malapaka`  | `Bharadwaj Chukkala` | `Kumara Ritvik Oruganti` |
| ------------- | ------------- |------------- |
| fill cheyali  | 118341705 | fill cheyali |
| Navigator | Driver |  Design Keeper |\

#### Member Intros

---

## Content Tree

---

## Software Project Management Plan Aspects

- ```Process, Tools and Technologies, Risk Management, References``` - [Project Proposal]()
- ```Design, Initial Product Backlog (ESC), Assigning Responsibilities (CRC)``` - [Software Development utils]()
- ``` Implementation, Class Diagram, Activity Diagram``` - [UML Diagrams]()

---

## Deliverables

- Project: Collection Robot
- Overview of proposed work including timeline, risks, and mitigations.
- UML diagrams
- Github repository with [README](./readme.md)
- GitHubCI setup with code coverage using Coveralls.
- Valgrind Check for Memory Leaks.
- Git Version Control Workflow.
- Developer-level documentation.

---

## Results

---

## Development Aspects

Agile Iterative Development Process will be used to develop the software along Test-Driven Development.

### [Product Backlog and Sprint Sheet](https://docs.google.com/spreadsheets/d/1XZ4UL9ePxGej_Jj6zL2NBFaGKRe57Hb5tn70BDvRW7s/edit?usp=sharing)

### [Sprint Review Sheet](https://docs.google.com/document/d/1JswTIkvGNDT8kqs0M8pbFIWWatNoKk5qQq9jqdAfdCM/edit?usp=sharing)

### Software Dependencies

- ros2 (covered under the open-source Apache 2 License)
- OpenCV 4.6.0 (covered under the open-source Apache 2 License)
- GTest BSD 3-Clause "New" or "Revised" License

### Tools and Technologies

`Ubuntu 20.04(LTS)/22.04` `C++ 14+` `CMake` `OpenCV` `TravisCI` `Coveralls`
`Makefile` `CMake` `cpplint` `cppcheck` `clangd` `Valgrind` `GTest` `VScode`

---

### Installation via Command Line

```
# Code Coverage
sudo apt-get install -y -qq lcov
```

```
# OpenCV install
sudo apt-get install -y build-essential
sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
# Download v4.6.0
curl -sL https://github.com/Itseez/opencv/archive/4.6.0.zip > opencv.zip
unzip opencv.zip
cd opencv-4.6.0
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
cd ../../
```

``` 
# Static Code Analysis
sudo apt install cpplint
sudo apt install cppcheck
```

```
# Valgrind
sudo apt install valgrind
sudo apt-get install -y kcachegrind
```

```
# Doxygen
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```

---

### Build, Run and Code Coverage

```
# Clone
git clone https://github.com/bharadwaj-chukkala/ENPM808X-Final-Project.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
# Run
Run tests: ./test/cpp-test
Run the perception module: ./app/shell-app
```

```
# Static Code Analysis
1. cppcheck
bash run_cppcheck.sh
2. cpplint
bash run_cpplint.sh
```
Note: Static Code Analysis Results are stored in `./results`

```
# Valgrind
valgrind --leak-check=full <path of the executable>
valgrind --tool=callgrind  ./app/shell-app
kcachegrind
```

```
# Code Coverage
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
cd coverage
firefox index.html
```

### Doxygen Documentation

```
cd ..
doxygen doxygen.config
doxywizard
```

---

## Project Videos

### [Final Video]()

### [Phase 2 Video]()

### [Phase 1 Video]() -->
