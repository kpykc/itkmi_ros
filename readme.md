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

Add to `CMakeLists.txt` добавить:

~~~{.cmake}
#.....
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
#.....
add_executable(ImageRegistration13 src/ImageRegistration13.cxx) 
target_link_libraries(ImageRegistration13 ${ITK_LIBRARIES})
~~~

