# Data communication

This is the general communication protocol for the whole UGV/UAV system.

## Installation
```bash
git clone $repository_address
sudo apt install bluez
```

### Build the following on all devices
```bash
g++ main.cpp comms.cpp -o communication -lbluetooth -lpthread
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
```
Then run: 
```bash 
./communication.
```
Preferrably set this up so the program starts when the device boots.
