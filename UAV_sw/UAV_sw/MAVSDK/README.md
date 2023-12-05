# MAVSDK guide

## Installation

### Get prerequisites

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git pip
```

### Clone the source

Note: after cloning, all other commands are within the cloned MAVSDK directory

```bash
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git submodule update --init --recursive
```

### Build the source

Note: -j4 means that 4 cpu cores will be used for building. Change to what you want.

```bash
cmake -DCMAKE_BUILD_TYPE=Release -Bbuild/default -H.
cmake --build build/default -j4
```

### Install the build

```bash
sudo cmake --build build/default --target install
```

## Building and running an example


First head over to examples directory.

```bash
cd examples/
```

Then go into desired example project, for example takeoff_and_land.

```bash
cd takeoff_and_land/
```

Then you can build it with these commands.

```bash
cmake -Bbuild -H.
cmake --build build -j4
```

And you can run it with this command.

```bash
build/takeoff_and_land udp://:14540
```