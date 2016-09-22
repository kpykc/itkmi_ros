# readme

Dependencies:

- opencv
- itk
- boost

Optional, package configuration depends on this libs, but code currently not:

- eigen3
- zlib
- pcl

Note: itk opencv bridge has problems with C++11.

## ITK

~~~{.bash}
brew install homebrew/science/insighttoolkit  --with-opencv
wget https://raw.githubusercontent.com/InsightSoftwareConsortium/ITK/master/Examples/RegistrationITKv4/ImageRegistration13.cxx
~~~

Add to `CMakeLists.txt`:

~~~{.cmake}
#.....
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
#.....
add_executable(ImageRegistration13 src/ImageRegistration13.cxx)
target_link_libraries(ImageRegistration13 ${ITK_LIBRARIES})
~~~

```
sudo apt install librosbag-dev  librosbag-storage-dev python-rosbag \
ros-kinetic-rosbag  ros-kinetic-rosbag-storage ros-kinetic-rosbaglive \
python-image-geometry ros-kinetic-image-geometry libimage-geometry-dev \
ros-kinetic-cmake-modules extra-cmake-modules

```