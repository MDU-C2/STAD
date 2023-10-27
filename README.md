# Data communication

This is the general communication protocol for the whole UGV/UAV system.

## Installation
```bash
git clone $repository_address
sudo apt install bluez
sudo apt-get install libbluetooth-dev
```

### Build the following on all devices
```bash
g++ main.cpp comms.cpp -o communication -lbluetooth -lpthread
```
On Ubuntu 16.04, build with following:
```bash
g++ -std=c++11 main.cpp comms.cpp -o px2 -lbluetooth -lpthread
```

## Usage
Uncomment the configuration file depending on what device the program will run on

```python
# Configuration File
#device=itx
#device=px2
device=drone

# Use this one together with device drone
remote_connection=XX:XX:XX:XX:XX:XX

# Use this one together with device itx
# remote_connection=XX:XX:XX:XX:XX:XX

# Use this one together with device px2
#remote_connection=XXX.XXX.XXX.XXX
```
Then run: 
```bash 
./communication.
```
Preferably set this up so the program starts when the device boots.
