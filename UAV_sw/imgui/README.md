# ImGui guide

## Info

This description is taken from the origional repo.

Dear ImGui is a bloat-free graphical user interface library for C++. It outputs optimized vertex buffers that you can render anytime in your 3D-pipeline-enabled application. It is fast, portable, renderer agnostic, and self-contained (no external dependencies).

Dear ImGui is designed to enable fast iterations and to empower programmers to create content creation tools and visualization / debug tools (as opposed to UI for the average end-user). It favors simplicity and productivity toward this goal and lacks certain features commonly found in more high-level libraries.

Dear ImGui is particularly suited to integration in game engines (for tooling), real-time 3D applications, fullscreen applications, embedded applications, or any applications on console platforms where operating system features are non-standard.

## Cloning the repository

Notice the "-b docking" in the clone command. We want this specific branch.

```bash
git clone -b docking https://github.com/ocornut/imgui.git
```

Do NOT cd into the directory.

## Moving the files

We want to move the .h files to a include directory and the .cpp files to a lib directory. Run the commands below to do that.

Note that it is not recursive. Do NOT use -r.

The following commands gets the source code for imgui as well as the glfw and OpenGL backend used for rendering.

```bash
cp imgui/*.h src/imgui/
cp imgui/*.cpp src/imgui/
cp imgui/backends/imgui_impl_glfw.h src/imgui/
cp imgui/backends/imgui_impl_glfw.cpp src/imgui/
cp imgui/backends/imgui_impl_opengl3.h src/imgui/
cp imgui/backends/imgui_impl_opengl3.cpp src/imgui/
cp imgui/backends/imgui_impl_opengl3_loader.h src/imgui/
```

The setup is complete here.

## Compiling

These files are not compiled by them selves. Instead they are included in other projects. See the CMakeLists.txt in the uav_gui/ directory.