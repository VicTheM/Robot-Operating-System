# Robot-Operating-System
Code snippets used during a study of  ROS 2. Majorly intermediary concepts<br>

#### A FEW CONCEPTS TO NOTEI

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

**This repo is organized as follows**
- two top level folders, both exclusively dedicated to rlcpy and rclcpp. I tried as much as possible to cover
all ROS2 inbuilt concepts in both folders. Unfortunately, some things can only be done in cpp and not python, so
they are not included in the python folder, and if other pythin things are dependent on them, then a little cpp will
be written in the python folder. Example of such things are creating custiom messages

- Almost every sub-folder has a readme file explaining what is in each folder, so a parent folder will only explain
its immediate directory and not that of the subfolders

- Packages are named based on concepts that they cover

- Nodes are not named based on concepts they cover
