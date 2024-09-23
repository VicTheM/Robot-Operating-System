# Robot-Operating-System
Code snippets used during a study of  ROS 2. Majorly intermediary concepts<br>

### A FEW CONCEPTS TO NOTE
<p>A workspace is a folder that contain packages, and all the source for these
packages will be housed in a subfolder called src. All sibling folder to the src
folder ie (build, install and log) are generated automatically by the colcon build
command<p>
<p>A packages is a self contained unit of code that houses all it needs and can be shared.
Packages uses ament build system and colcon tool. packages can be built using either CMake
or Python. A package is simply a folder in the src folder<p>

#### EXAMPLE OF HOW A SIMPLE WORKSPACE COULD BE

workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
