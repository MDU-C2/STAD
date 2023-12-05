# PX4-Autopilot guide

## Installation

### Clone the source


```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
```

## Build source

```bash
make px4_sitl
```

## Build and run simulation

Note: yes, this command is used for both building AND running

```bash
make px4_sitl gazebo-classic
```