# Visual_Odometry
Progetto SGN: Visual Odometry

L'obiettivo del progetto è quello di implementare uno script MATLAB di Visual Odometry su ROS, con nodi implementati in C++.
## Guida per l'installazione di OpenCV, opencv_contribe cv_bridge
Per l'installazione di OpenCV 4 con opencv_contrib, digitare nel terminale:

```
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```
Queste 3 istruzioni creano una cartella `opencv_build` dentro cui vengono scaricati `opencv` e `opencv_contrib`.

Dopo di ciò, dal terminale:

```
cd ~/opencv_build/opencv
mkdir build && cd build
```
Setup OpenCV build con CMake:

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \  
        -D CMAKE_INSTALL_PREFIX=/usr/local \  
        -D INSTALL_C_EXAMPLES=ON \  
        -D INSTALL_PYTHON_EXAMPLES=ON \  
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \  
        -D BUILD_EXAMPLES=ON .. \  
        -D OPENCV_ENABLE_NONFREE=ON ..
```
Compiliamo:

```
make -j8
```
Installiamo:

```
sudo make install
```
L'installazione è completa.


Disinstallare OpenCV:

```
sudo rm /usr/local/{bin,lib}/*opencv* 
```

E' necessario inoltre avere il codice sorgente di `vision_opencv` ed in particolare di `cv_bridge`. Infatti, le librerie di OpenCV 4 e quella interna a ROS andrebbero in conflitto. Per risolvere questo problema, si installa un particolare `vision_opencv`, in questo modo:

```
cd ~/catkin_ws/src
git clone https://github.com/fizyr-forks/vision_opencv.git
cd vision_opencv
git checkout opencv4
```

### Branch
La repository di Visual Odometry ha 2 branch. 
- Main: Per la bag Rapallo.
- Master:Per le bags Parete e Cilindro.