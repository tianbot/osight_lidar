# osight_lidar
ROS driver for Osight 2D lidar, supported by Tianbot

Osight Lidar IExxx
Purchase link(淘宝购买链接)：https://item.taobao.com/item.htm?id=612044849834

## Specification
|  Items  | Details|
|    ---    |     ---     |
| Model  | IE103-S |
| Environment  | Outdoors |
| Working temperature  | -20℃～+60℃ |
| IP  | IP67 |
| Interface  | Ethernet |
| Laser safety  | class 1 |
| Distance  | 0.3～30m |
| Scan frecuency  | 10Hz、20Hz、30Hz、50Hz |
| Angular resolution  | 0.125°、0.25°、0.5° |
| Angle range  | 270° |
| Intensities  | Yes |
| Accuracy  | ≤±30mm |
| Protocol  | UDP |
| Input voltage  | 10～28V DC |
| Weight  | 320 g |
| Size  | 60mm×60mm×93mm |
| Power Consumption  | ≤ 4W |

Standard:  
IEC 60068-2-27:2008  
IEC 60068-2-6:2007  
EMC EN 61000-6-2:2005  
EN 61000-6-4:2007+A1  


## Installation Instructions
You can skip these steps if you purchase Osight Lidar with a ROS2GO system 
- Install from debian package  
  To be released
- Install from source

Steps to install to catkin_ws. 
```
cd ~/catkin_ws/src/
git clone https://github.com/tianbot/osight_lidar.git
cd ~/catkin_ws && catkin_make
```
## Usage Instructions
Make sure the power supply and ehternet connection are correct.

### Start osight_lidar node
Launch the lidar.

```
roslaunch osight_lidar osight_lidar.launch
```

## License
The package is under BSD 3-Clause License
