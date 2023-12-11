# Micro-XRCE-DDS-Agent

Having the px4 messages and ros2 is not enough for ros communication with the flight controller.
Micro-XRCE-DDS-Agent is a small software acting as a link, and all the px4 messages will communicate with the flight controller indirectly via this program.

### Installation

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git 
cd Micro-XRCE-DDS-Agent 
mkdir build 
cd build 
cmake .. 
Make 
sudo make install 
sudo ldconfig /usr/local/lib/ 
```

To use this, the px4 flight controller requires the raspberry pi to be connected to a specific ip address. Here is how to set it up:

### Setup network config: Flight Controller

Guide to set up Ethernet, ROS2 between FC and MC:
 
In QGroundcontrol upgrade FC firmware to 1.14 so uXRCE-DDS can be configured. 
 
In MAVLink Console, do the following:

```bash
echo DEVICE=eth0 > /fs/microsd/net.cfg 
echo BOOTPROTO=static >> /fs/microsd/net.cfg //Different command from general px4 guide 
echo IPADDR=192.168.0.4 >> /fs/microsd/net.cfg 
echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg 
echo ROUTER=192.168.0.254 >>/fs/microsd/net.cfg 
echo DNS=192.168.0.254 >>/fs/microsd/net.cfg 
```

Change the following parameters in VEHICLE setup: 
 
#### UXRCE_DDS_CFG: Ethernet 
#### UXRCE_DDS_AG_IP: 3232235521 (this is 192.168.0.1 in decimal format. The FC client connects to this IP) 
 
Reboot the FC.
 
After reboot, verify the settings in MAVLink console with the following commands:

```bash
netman show 
uxrce_dds_client status 
```

### Setup network configuration: Raspberry Pi

In Ubuntu: Add the following in /etc/netplan/<config file>.yaml 
careate a backup from previous file.


```bash
cp /etc/netplan/<config file>.yaml /etc/netplan/<config file>.yamlBack 
```

edit the /etc/netplan/<config file>.yaml  and copy the following value.  

```bash
network: 
  version: 2 
  renderer: NetworkManager 
  ethernets: 
      eth0: 
          addresses: 
              - 192.168.0.1/24 
          nameservers: 
              addresses: [192.168.0.1] 
          routes: 
              - to: 192.168.0.1 
                via: 192.168.0.1
```

To change network, run:

```bash
sudo netplan apply 
```

Connect Ethernet cable between FC and RPi. 
 
From RPi: ping 192.168.0.4 
From MAVLink console in QGroundcontrol: ping 192.168.0.1 

 
### Verify ROS2 functionality 
 
On RPi run the following in one terminal:

```bash
MicroXRCEAgent udp4 -p 8888 
```

And this in another terminal:

```bash
source ~/px4_ros2_ws/install/setup.bash 
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

You should receive sensor data in the second terminal.
