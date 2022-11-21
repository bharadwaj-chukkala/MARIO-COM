# Medical Trash Collection Robot

[![Build Status](https://app.travis-ci.com/spauly98/ENPM808X_Midterm_project.svg?branch=phase2)](https://app.travis-ci.com/github/bharadwaj-chukkala/ENPM808X_Midterm_project)
[![Coverage Status](https://coveralls.io/repos/github/spauly98/ENPM808X_Midterm_project/badge.svg?branch=phase2)](https://coveralls.io/github/spauly98/ENPM808X_Midterm_project?branch=phase2)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Overview

### Problem Statement
<---------- Problem Statement goes here----------->

### Goal
- <------ What are the software developement goals----->
- <------ Product Scalabaility to further acheive ambitious goals------>
- <------ Product Software Fucntionlaity oursourcing --------> 

### Aim of the Product
- <-------- What is  the product supposed to do------>
- <-------- How is it going to betteer the lives of customers-------->

### Product Description
<------- Aspects of the product ------->


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
### [Phase 1 Video]()

