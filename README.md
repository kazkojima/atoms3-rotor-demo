## M5ATOM S3 ESP-IDF sample

This is a sample application of M5ATOM S3 using IMU(MPU6886).
MPU6886 driver is taken and modified from the [original arduino M5ATOM library](https://github.com/m5stack/M5Atom).
For LCD, [LovyanGFX](https://github.com/lovyan03/LovyanGFX.git) is used as a submodule. AHRS is a geometric algebra based 3D rotor identification which is essentially a complementary filter.

It displays AHRS result with a simple xyz frame.
