# UAV_GUI guide

This is a graphical user interface (GUI) for the UAV

### Before compiling

Note: This code requires both glfw and imgui. You must go into those folders and do those steps first.

### Compile

After changing the code you can build it with these commands. Do it from the uav_gui folder, not src folder.

```bash
cmake -Bbuild -H.
cmake --build build -j4
```

And you can run it with this command.

```bash
build/uav_gui
```