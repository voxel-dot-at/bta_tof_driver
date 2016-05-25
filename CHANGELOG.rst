^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bta_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2016-05-25)
------------------

0.0.6 (2016-05-18)
------------------
* Improved performance. Added fixed transformation for cloud. Changed frameid names, converting xyz values to meters
* added launch configuration for displaying 3D pointclouds. use 'roslaunch bta_ros node_tof_3d.launch' to start up
* updated IP parameters to reflect default
  updated IP parameters to reflect default camera settings.
* Update README.md
* Updated CHANGELOG.rst v0.0.5
* Contributors: Angel Merino, Simon Vogl

0.0.5 (2016-04-13)
------------------
* Added launch and config files for TIM-ETH interface
* Added support for BtaTofApi v2.1
* Added Jon Azpiazu's launch file for displaying depth data
* fixed step calculation (Thanks Jon Azpiazu)
* Merge branch 'master' of https://github.com/voxel-dot-at/bta_ros
* Fixed 2DSENSOR flag default
* Update README.md
* Contributors: Angel Merino

0.0.4 (2014-11-10)
------------------
* Updated Changelog
* Updated Changelog
* Added capture of 3d data and publish as pointcloud2 when possible
* Merge branch 'master' of https://github.com/voxel-dot-at/bta_ros
* Updated and added launch files and parameters for P100
* Added library selection options and messages
* Added pointcloud topic and logic to publish in them depending on frame type
* Added Voxel.at address
* Updated ip to url in launch files
* Changed ip to url
* Added 2DSENSOR flag to activate rgb sensor compile and include gstreamer
* Added cmake flats for selecting bta library: BTA_ETH or BTA_P100
* added /usr/local/include to the search path
* Update README.md
* Added register operations read/write console messages
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Updated README.md
* Contributors: Angel Merino, Simon Vogl

0.0.3 (2014-11-05)
------------------
* Added funtion to read and write register in dynamic_reconfigure
* Updated changelog
* Added 2d video support and minor fixed
* Updated readme
* Contributors: Angel Merino

0.0.2 (2014-11-04)
------------------
* 0.0.1
* UDP address back to default
* First release
* 0.0.1
* Updated CMakeList.txt
* First commit
* Initial commit
* Contributors: Angel Merino, Simon Vogl
