# Crazyflie Firmware  [![CI](https://github.com/bitcraze/crazyflie-firmware/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie-firmware/actions?query=workflow%3ACI)

This project contains the source code for the firmware used in the Crazyflie range of platforms, including the Crazyflie 2.x and the Roadrunner.

### Changes

App layer (Spring Tension Estimation)
-. crazyflie-firmware/examples/app_fs/src/app_fs.c

add Log :
- Fsx : Spring Tension Estimation value x
- Fsy : Spring Tension Estimation value y
- Fsz : Spring Tension Estimation value z

- FTwx : Thrust Estimation value x
- FTwy : Thrust Estimation value y
- FTwz : Thrust Estimation value z

- maw_dx : mass * acceleration value x 
- maw_dy : mass * acceleration value y 
- maw_dz : mass * acceleration value z

### Complie & build.

cd crazyflie-firmware/examples/app_fs : Move to the app layer.

make clean : Erase previous debris.

make : Build it completely from scratch.

make cload : The finished product is baked into the crazyflie.
