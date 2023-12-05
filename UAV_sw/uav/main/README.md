# UAV guide

This is the main drone code for controlling the drone. This is what to change when developing.

### Compile

After changing the code you can build it with these commands. Do it from the uav folder, not src folder.

```bash
cmake -Bbuild -H.
cmake --build build -j4
```

And you can run it with this command.

```bash
build/uav
```