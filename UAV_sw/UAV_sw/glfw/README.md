# GLFW guide

## Info

This description is taken from the origional repo.

GLFW is an Open Source, multi-platform library for OpenGL, OpenGL ES and Vulkan application development. It provides a simple, platform-independent API for creating windows, contexts and surfaces, reading input, handling events, etc.

GLFW natively supports Windows, macOS and Linux and other Unix-like systems. On Linux both X11 and Wayland are supported.

## Cloning repository

Run this command from current directory.

```bash
git clone https://github.com/glfw/glfw.git --recursive
```

## Platform detection

These features are platform specific so you must know which windowing system you are currently using (X11, xorg, wayland). You can do that with any of the following commands. Run them and see what is returned. X11 and xorg is the same thing.

```bash
echo "$XDG_SESSION_TYPE"
env | grep -E -i 'x11|xorg|wayland'
loginctl show-session $(loginctl | grep "$USER" | awk '{print $1}') -p Type
loginctl show-session $(awk -v u="$USER" '$0 ~ u{ print $1}'<<<$(loginctl)) -p Type
loginctl show-session $(awk -v u="$USERNAME" '$0 ~ u{ print $1}'<<<$(loginctl)) -p Type
```

## Setup guide

If you have a debian based ubuntu 20.04 edition, this guide should be straight forward. Otherwise you might get problems. Look at the [original guide](https://www.glfw.org/docs/latest/compile.html) to trouble shoot in that case. You can find out if it is debian with this command.

```bash
cat /etc/*-release
```

#### Note: You either use X11 OR Wayland. Not both.

### X11

Install X11 dependencies with this command.

```bash
sudo apt install xorg-dev
```

### Wayland

Install Wayland dependencies with this command.

```bash
sudo apt install libwayland-dev libxkbcommon-dev wayland-protocols extra-cmake-modules
```

## Building

From this point on everything will be done in the repository so it is very important you are in that folder.

```bash
cd glfw
```

When in the correct directory you build using this command.

### X11

For X11 use this.

```bash
cmake -S . -B build
```

### Wayland

For Wayland use this.

```bash
cmake -S . -B build -D GLFW_USE_WAYLAND=1
```

## Installing

Head over to build folder

```bash
cd build
```

Build using this command.

```bash
sudo make install
```

Notice that we do "sudo make install" instead of "make" because we want this library to be available everywhere. Not just in this folder.

After this the library should be compiled and be ready for deployment