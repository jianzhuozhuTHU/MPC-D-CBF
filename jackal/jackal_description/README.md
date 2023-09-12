# Jackal Description

This packages contains the meshes and URDF of the Jackal robot, its supported sensors, and their supported mounts. 

# Jackal Payloads

## Microstrain IMU
```bash
export JACKAL_IMU_MICROSTRAIN=1
```
###### Launch
```bash
export JACKAL_IMU_MICROSTRAIN_NAME="microstrain"
```
###### Description
```bash
export JACKAL_IMU_MICROSTRAIN_LINK="microstrain_link"
export JACKAL_IMU_MICROSTRAIN_PARENT="base_link"
export JACKAL_IMU_MICROSTRAIN_OFFSET="-0.139 0.096 0.100"
export JACKAL_IMU_MICROSTRAIN_RPY="3.14159 0 -1.5707"
```

## 2D Laser
#### Primary
```bash
export JACKAL_LASER=1
export JACKAL_LASER_MODEL=lms1xx # or tim551 or ust10 or utm30
```
###### Launch
```bash
export JACKAL_LASER_TOPIC="front/scan"
export JACKAL_LASER_HOST="192.168.131.20"
```
###### Description
```bash
export JACKAL_LASER_TOWER=1
export JACKAL_LASER_MOUNT="front"
export JACKAL_LASER_PREFIX="front"
export JACKAL_LASER_PARENT="front_mount"
export JACKAL_LASER_MOUNT_TYPE="upright" # or "inverted"
export JACKAL_LASER_OFFSET="0 0 0"
export JACKAL_LASER_RPY="0 0 0"
```
#### Secondary
```bash
export JACKAL_LASER_SECONDARY=1
export JACKAL_LASER_SECONDARY_MODEL="lms1xx" # or "tim551" or "ust10" or "utm30"
```
###### Launch
```bash
export JACKAL_LASER_SECONDARY_HOST="192.168.131.21"
export JACKAL_LASER_SECONDARY_TOPIC="rear/scan"
```
###### Description
```bash
export JACKAL_LASER_SECONDARY_TOWER=1
export JACKAL_LASER_SECONDARY_MOUNT="rear"
export JACKAL_LASER_SECONDARY_PREFIX="rear"
export JACKAL_LASER_SECONDARY_PARENT="rear_mount"
export JACKAL_LASER_SECONDARY_OFFSET="0 0 0"
export JACKAL_LASER_SECONDARY_RPY="0 0 3.13159"
```

## 3D Laser
```bash
export JACKAL_LASER_3D=1
```
###### Launch
```bash
export JACKAL_LASER_3D_HOST="192.168.131.20"
export JACKAL_LASER_3D_TOPIC="mid/points"
```
###### Description
```bash
export JACKAL_LASER_3D_TOWER=1
export JACKAL_LASER_3D_MOUNT="mid"
export JACKAL_LASER_3D_PREFIX="mid"
export JACKAL_LASER_3D_PARENT="mid_mount"
export JACKAL_LASER_3D_MODEL="vlp16" # or "hdl32e"
export JACKAL_LASER_3D_OFFSET="0 0 0"
export JACKAL_LASER_3D_RPY="0 0 0"
```

## NAVSAT
```bash
export JACKAL_NAVSAT=1
```
###### Launch
```bash
export JACKAL_NAVSAT_PORT="/dev/clearpath/gps"
export JACKAL_NAVSAT_BAUD=57600
export JACKAL_NAVSAT_RTK=0
export JACKAL_NAVSAT_RTK_DEVICE=wlan0
export JACKAL_NAVSAT_RTK_BAUD=57600
```
###### Description
```bash
export JACKAL_NAVSAT_TOWER=1
export JACKAL_NAVSAT_HEIGHT=0.1 # in meters
export JACKAL_NAVSAT_MOUNT="rear"
export JACKAL_NAVSAT_PREFIX="rear"
export JACKAL_NAVSAT_PARENT="rear_mount"
export JACKAL_NAVSAT_MODEL="smart6" # or "smart7"
export JACKAL_NAVSAT_OFFSET="0 0 0"
export JACKAL_NAVSAT_RPY="0 0 0"
```

## Pointgrey Flea3
```bash
export JACKAL_FLEA3=1
```
###### Launch
```bash
export JACKAL_FLEA3_SERIAL=0
export JACKAL_FLEA3_FRAME_RATE=30
export JACKAL_FLEA3_CALIBRATION=0
```
###### Description
```bash
export JACKAL_FLEA3_TOWER=1
export JACKAL_FLEA3_TILT=0.5236
export JACKAL_FLEA3_NAME="front"
export JACKAL_FLEA3_MOUNT="front"
export JACKAL_FLEA3_PREFIX="front"
export JACKAL_FLEA3_PARENT="front_mount"
export JACKAL_FLEA3_OFFSET="0 0 0"
export JACKAL_FLEA3_RPY="0 0 0"
```

## Stereo Pointgrey Flea3
```bash
export JACKAL_STEREO_FLEA3=1
```
###### Launch
```bash
export JACKAL_FLEA3_FRAME_RATE=30
export JACKAL_FLEA3_LEFT_SERIAL=0
export JACKAL_FLEA3_LEFT_CALIBRATION=0
export JACKAL_FLEA3_RIGHT_SERIAL=0
export JACKAL_FLEA3_RIGHT_CALIBRATION=0
```
###### Description
```bash
export JACKAL_STEREO_SEPERATION=0.16
export JACKAL_FLEA3_TILT="0.5236"
export JACKAL_FLEA3_MOUNT="front"
export JACKAL_FLEA3_PREFIX="front"
export JACKAL_FLEA3_PARENT="front_mount"
export JACKAL_FLEA3_LEFT_NAME="front/left"
export JACKAL_FLEA3_RIGHT_NAME="front/right"
export JACKAL_FLEA3_OFFSET="0 0 0"
export JACKAL_FLEA3_RPY="0 0 0"
```

## Bumblebee2
```bash
export JACKAL_BB2=1
```
###### Launch
```bash
export JACKAL_BB2_SERIAL=0
export JACKAL_BB2_CALIBRATION=0
```
###### Description
```bash
export JACKAL_BB2_TILT=0
export JACKAL_BB2_TOWER=1
export JACKAL_BB2_NAME="front"
export JACKAL_BB2_MOUNT="front"
export JACKAL_BB2_PREFIX="front"
export JACKAL_BB2_PARENT="front_mount"
export JACKAL_BB2_OFFSET="0 0 0"
export JACKAL_BB2_RPY="0 0 0"
```

## Flir Blackfly
```bash
export JACKAL_BLACKFLY=1
```
###### Launch
```bash
export JACKAL_BLACKFLY_SERIAL=0
export JACKAL_BLACKFLY_DEVICE="USB" # or "GigE"
export JACKAL_BLACKFLY_ENCODING="BayerRGB"
export JACKAL_BLACKFLY_FRAMERATE=30
```
###### Description
```bash
export JACKAL_BLACKFLY_PREFIX="front_camera"
export JACKAL_BLACKFLY_PARENT="front_mount"
export JACKAL_BLACKFLY_OFFSET="0 0 0"
export JACKAL_BLACKFLY_RPY="0 0 0"
```

## Front and Rear Accessory Fender
```bash
export JACKAL_FRONT_ACCESSORY_FENDER=1
export JACKAL_REAR_ACCESSORY_FENDER=1
export JACKAL_FRONT_FENDER_UST10=1
export JACKAL_REAR_FENDER_UST10=1
```
###### Launch
```bash
export JACKAL_FRONT_LASER_TOPIC=front/scan
export JACKAL_FRONT_LASER_HOST="192.168.131.20"
export JACKAL_REAR_LASER_TOPIC=rear/scan
export JACKAL_REAR_LASER_HOST="192.168.131.21"
```

## Decorations
Description only accessories (i.e. no driver).
###### Wibotic Q-Charging Bumper:
```bash
export JACKAL_WIBOTIC_BUMPER=1
```
###### Backpack Computer Enclosure:
```bash
export JACKAL_ARK_ENCLOSURE=1
```