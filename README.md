# Complementrary SLAM use with rovio and leg odometry

### Set up GTSAM 4.0.3 to work with compslam ###

Clone repository:
```
$ git clone git@github.com:borglab/gtsam.git -b 4.0.3
```

There are options for globally or locally installing gtsam on your system, follow either set of steps.

#### Global install ####

Install with required flags:
```
$ cd <your_path_to_gtsam>/gtsam
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF ..
$ make install
```

#### Local install ####

Install with required flags:
```
$ cd <your_path_to_gtsam>/gtsam
$ mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX:PATH=<your_local_installation_path> -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF ..
$ make install
```

Link to locally installed libs:
```
$ echo -e '\n# Link to local GTSAM installation libs' >> ~/.bashrc
$ echo -e 'export LD_LIBRARY_PATH=<your_installation_path>/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
```

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact