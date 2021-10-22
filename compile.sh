opencv_installed=0

# Check if OpenCV 4 is installed
opencv=$(pkg-config --modversion opencv4)
if [ ${opencv:0:1} == 4 ]
then
    opencv_installed=1
else
    echo OpenCV4 v4 not installed. Do you want to install OpenCV4? \(y/n\)
    read install_opencv
    if [ $install_opencv == y ] || [ $install_opencv == Y ]
    then
        # Install OpenCV 4 (http://www.codebind.com/linux-tutorials/install-opencv-ubuntu-18-04-lts-c-cpp-linux/)
        echo Installing...
        dir=$(pwd)

        # Updating Ubuntu
        sudo apt-get update
        sudo apt-get upgrade

        # Install dependencies
        sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
        sudo apt-get install python3.5-dev python3-numpy libtbb2 libtbb-dev
        sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev

        # Get OpenCV
        # sudo -s
        cd /opt
        git clone https://github.com/Itseez/opencv.git
        git clone https://github.com/Itseez/opencv_contrib.git

        # Build and install OpenCV
        cd opencv
        mkdir release
        cd release
        sudo cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv/
        sudo make -j4
        sudo make install
        sudo ldconfig
        cd $dir

        # Test to see if OpenCV 4 is installed
        opencv=$(pkg-config --modversion opencv4)
        if [ ${opencv:0:1} == 4 ]
        then
            echo OpenCV successfully installed!
            opencv_installed=1
        else
            echo OpenCV was not successfully installed. Please check.
        fi
    fi    
fi

if [ $opencv_installed == 1 ]
then
    g++ -I./include -L./lib/hungarian -L./lib/json src/*.cpp lib/hungarian/Hungarian.cpp lib/json/jsoncpp.cpp -o spot-it-3d `pkg-config --cflags --libs opencv4 gstreamer-1.0`
    # ./spot-it-3d
fi