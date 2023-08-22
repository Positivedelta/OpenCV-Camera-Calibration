# OpenCV Camera Calibration

#### Notes
- Written using C++17
- This project requires OpenCV, if installed OpenCV should be detected automatically by CMake

#### Build
```
mkdir build
cd build
cmake ..
```

#### Install Instructions
To install the application use:

```
make install
```

The default install path is `bit-parallel/bin/camera`, edit `CMakeLists.txt` to change this to something more useful

#### Execute
```
./camera-calibration-bp-v1.0 -c [#camera] [image capture directory]
./camera-calibration-bp-v1.0 -c [#camera] [image capture directory] -d
./camera-calibration-bp-v1.0 -t [#camera]
```

- You must create an image capture directory before performing a calibration
- Use the `-d` option to delete the contents of the image capture directory
- Use the `-t` option to test the resulting calibration using a live camera feed
- The resulting calibration data is stored in `calibration.xml` in the current directory
