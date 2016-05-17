Multi-Kinect Autocalibration
=============================

This program is used to get extrinsic calibration matrizes for a multi-kinect 
setup. It searches for a sphere in three dimensional space and remembers the 
position of the spheres center in each camera space. After a minimum of three 
points have been collected, a transformation estimation is performed to get the 
found positions of the sphere to overlap. The resulting translation vector and 
rotation quaternion can be saved to an xml file.

Requirements
==============

* openFrameworks 	 	9.0.0 	(https://github.com/openframeworks/openFrameworks)
* Poincloud Library 	1.8.0 	(https://github.com/PointCloudLibrary/pcl)
* of-pcl-bridge					(https://github.com/iwer/of-pcl-bridge)
* librecon						()