Agricultural Robotics 3.0

Directory structure:
	Production		# Deployable program code
	- images		# Images used by the program

	Testing			# Test scripts to verify new functionality
	- GUI
	- navigation
	- sensors
	- shapes
	- simulator
	- TestVids		# Videos of tests


Required programs:
	- Python 2
	- libav


Required Python 2 modules:
	- Scipy
	- Numpy
	- OpenCV3
	- TKinter
	- pillow


Installation
	- Install Python 2 and pip.
	- Update pip and setuptools.
	- Pip install Scipy, Numpy, TKinter, pillow.
	- Download OpenCV3 source.
	- CMake using Python2 and Numpy directories (details below).
	- Verify Video: FFmpeg support.
	- Make + Make Install OpenCV.

To run:
	- Connect to ARDrone wifi signal.
	- Run AgriDrone.py with Python2.


OpenCV sample cmake command:

cmake \
-D CMAKE_BUILD_TYPE=Release \
-D PYTHON2_EXECUTABLE=/usr/bin/python2.7 \
-D PYTHON_INCLUDE_DIR=/usr/include/python2.7 \
-D PYTHON_LIBRARY=/usr/lib/python2.7 \
-D PYTHON2_NUMPY_INCLUDE_DIRS=/usr/include/python2.7/numpy \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_EIGEN=OFF \
-D WITH_1394=OFF \
..

Note:
These defines should be included in the command, but the directories may
differ depending on Linux and Mac. Windows was not used and not recommended;
consider using a virtual machine with Ubuntu for development.

Verifying Installation:
The simplest way is to run Python 2 and attempt to import all required modules.
Additionally for OpenCV, run
	cv2.__version__
to be sure 3.x.x is displayed.

To verify video capture, run AgriDrone.py while connected to the drone's wifi
hotspot, click the "Connect" button, and check that the camera is displaying
a live feed.

