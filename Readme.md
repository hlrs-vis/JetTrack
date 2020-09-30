# JetTrack - MA_Intersection_Calculation

**MA_Intersection_Calculation** is JetTrack's main algorithmus for real time object position calculation.

## Overview:
  - ***Input-Data:*** Kafka Data Streams form the JetTrack - MA_DeepStream_Pipeline (running on each camera)
  - ***Output-Data***: Position of all objects in scene, which are detected by the deepstream pipeline

### Development
1) Start KafkaBroker on local Network @ 10.42.0.1:9092 or wherever you want
2) Start MA_DeepStream_Pipeline on JetsonNano Devices
3) Adust to your CameraParameters in src/config.json
    - Resolution of detection stream
    - Translation vector of camera (eg. see OpenCV camera calibration)
    - Rotation vector of camera (eg. see OpenCV camera calibration)
    - intrinsicMatrix
    - pathToCalibrationFile (for realtime use of camera calibration files - cam mode)
    - distCoeefs (k1, k2, p1, p2,[,k3,[,k4,k5,k6]]) (eg. see OpenCV camera calibration )
4) Run main.py on localDevice (connected to CamNetwork or on JetsonDevice itself)


For running Realtime Object Positioning:


- ***source:*** **local** -> src/config.json, **cams** -> settings on cams webserver will be used
- ***method:*** **aruco** -> realign aruco marker to world coordinate system


```sh
$ python3 main.py source method
```

### Todos
 - Add more calibrationModes for realingment of different CalibrationObjects
