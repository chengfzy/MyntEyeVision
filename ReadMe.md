# MYNT EYE Camera Example

## Install MYNT EYE SDK
1. My camera is MYNE-EYE-D1000-IR-120/Color.
1. `git clone` from [sdk](https://github.com/slightech/MYNT-EYE-D-SDK), and use cmake to build and install
1. The document said use `make init & make all`, the `make all` command could be replaced by cmake building.
1. The `samples` has many useful code and tools.

## Build this project
1. The sample in SDK show the device to obtain *distance* and *location(GPS)*, but the device could not obtain any value.

## Recorder
Recorder is used to same the image and IMU to folder.
1. Maybe lost some frame because save image in single thread.
1. The timestamp of accelerator and gyroscope are different. IMU的加速度计和陀螺仪时间戳不一致, 测试发现是Acc一个时间, Gyro一个时间.