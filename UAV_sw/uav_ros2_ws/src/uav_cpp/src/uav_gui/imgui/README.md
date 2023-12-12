# ImGui guide

This is a quick guide showing where the imgui files comes from.

Nothing has to be done further.

## Info

This description is taken from the origional repo:

Dear ImGui is a bloat-free graphical user interface library for C++. It outputs optimized vertex buffers that you can render anytime in your 3D-pipeline-enabled application. It is fast, portable, renderer agnostic, and self-contained (no external dependencies).

Dear ImGui is designed to enable fast iterations and to empower programmers to create content creation tools and visualization / debug tools (as opposed to UI for the average end-user). It favors simplicity and productivity toward this goal and lacks certain features commonly found in more high-level libraries.

Dear ImGui is particularly suited to integration in game engines (for tooling), real-time 3D applications, fullscreen applications, embedded applications, or any applications on console platforms where operating system features are non-standard.

## Cloning the repository and using the files

We already have the files but this is how you clone them.

Notice the "-b docking" in the clone command. We want this specific branch.

```bash
git clone -b docking https://github.com/ocornut/imgui.git
```

The following source files from the repository are needed from imgui so that the glfw and OpenGL backends can be used for rendering.

```bash
imgui/*.h
imgui/*.cpp
imgui/backends/imgui_impl_glfw.h
imgui/backends/imgui_impl_glfw.cpp
imgui/backends/imgui_impl_opengl3.h
imgui/backends/imgui_impl_opengl3.cpp
imgui/backends/imgui_impl_opengl3_loader.h
```