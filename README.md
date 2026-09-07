<h1 align="center">
  <a href="https://www.reactphysics3d.com"><img src="https://github.com/DanielChappuis/reactphysics3d/blob/62e17155e3fc187f4a90f7328c1154fc47e41d69/documentation/UserManual/images/ReactPhysics3DLogo.png" alt="ReactPhysics3D" width="300"></a>
  <br>
  ReactPhysics3D
  <br>
</h1>

<h4 align="center">ReactPhysics3D is an open source C++ physics engine library that can be used in 3D simulations and games.</h4>
<p align="center"><a href="https://www.reactphysics3d.com">www.reactphysics3d.com</a></p>
<p align="center">
  <a href="https://github.com/DanielChappuis/reactphysics3d/actions/workflows/build-and-test.yml">
    <img src="https://github.com/DanielChappuis/reactphysics3d/actions/workflows/build-and-test.yml/badge.svg"
         alt="Build">
  </a>
  <a href="https://www.codacy.com/app/chappuis.daniel/reactphysics3d?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=DanielChappuis/reactphysics3d&amp;utm_campaign=Badge_Grade"><img src="https://api.codacy.com/project/badge/Grade/3ae24e998e304e4da78ec848eade9e3a"></a>
  <a href="https://codecov.io/github/DanielChappuis/reactphysics3d?branch=master">
      <img src="https://codecov.io/github/DanielChappuis/reactphysics3d/coverage.svg?branch=master">
  </a>
</p>

<p align="center">
  <img src="https://github.com/DanielChappuis/reactphysics3d/blob/images/showreel.gif?raw=true" alt="Drawing" />
</p>

## Features

 - Rigid body dynamics
 - Discrete collision detection
 - Collision shapes (Sphere, Box, Capsule, Convex Mesh, Static Concave Mesh, Height Field)
 - Multiple collision shapes per body
 - Broadphase collision detection (Dynamic AABB tree)
 - Narrowphase collision detection (SAT/GJK)
 - Collision response and friction (Sequential Impulses Solver)
 - Joints (Ball and Socket, Hinge, Slider, Fixed)
 - Collision filtering with categories
 - Ray casting
 - Sleeping technique for inactive bodies
 - Multi-platform (Windows, Linux, Mac OS X)
 - No external libraries (do not use STL containers)
 - Documentation (user manual and Doxygen API)
 - Testbed application with demos
 - Integrated profiler
 - Debugging renderer
 - Logs
 - Unit tests

## Documentation

You can find the user manual and the Doxygen API documentation <a href="https://www.reactphysics3d.com/documentation" target="_blank">here</a>.

## Building

### Windows + MSYS2 MINGW64

Build from a **MINGW64** shell, not the MSYS shell.

- Install the toolchain. This must be the ```mingw-w64-x86_64``` CMake, *not* the ```msys``` one: the MSYS build reports a POSIX platform (```UNIX=1```, ```WIN32``` unset), which makes GLFW pick its X11 backend and fail the configure with ```Could NOT find X11```.

```pacman -Syu```
```pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-make```

```mingw-w64-x86_64-cmake``` depends on ```mingw-w64-x86_64-ninja```, so Ninja gets installed too and is CMake's default generator here. Run the ```-Syu``` first: MSYS2 does not support partial upgrades, and a freshly installed package built against a newer GCC will fail to start with a missing DLL entry point.

- If you want the testbed, checkout the nanogui submodule:

```git submodule update --init --recursive```

- Generate the build files. Swap in ```-G Ninja``` if you prefer:

```cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DRP3D_COMPILE_TESTBED=ON -DCMAKE_EXE_LINKER_FLAGS="-static -static-libgcc -static-libstdc++"```

- Then build with:

```CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake --build build -j```

The environment variable is only needed for the testbed. nanogui's ```resources/bin2c.cmake``` declares ```cmake_minimum_required (VERSION 2.8.12)``` and CMake 4 removed support for anything below 3.5. It has to be an environment variable rather than a ```-D``` flag because CMake runs that script through a nested ```cmake -P``` invocation, which a cache variable does not reach.

This produces the ```libreactphysics3d.a``` library plus ```build/testbed/testbed.exe```, statically linked against the MinGW runtime so it needs no MSYS2 DLLs to run.

Note that the ```.a``` is not ABI-compatible across GCC major versions. Anything linking it has to be built with the same MinGW GCC major version used here.

## Branches

The "master" branch always contains the last released version of the library and some possible hot fixes. This is the most stable version. On the other side,
the "develop" branch is used for development. This branch is frequently updated and can be quite unstable. Therefore, if you want to use the library in
your application, it is recommended to checkout the "master" branch.

## Questions

If you have any questions about the library and how to use it, you should use <a href="https://github.com/DanielChappuis/reactphysics3d/discussions" target="_blank">Github Discussions</a> to read previous questions and answers or to ask new questions. If you want, you can also share your project there if you are using the ReactPhysics3D library.

## Questions/Discussions

You can use the <a href="https://github.com/DanielChappuis/reactphysics3d/discussions" target="_blank">Discussions</a> section to ask your questions or answer questions from other users of the library. Please, do not open a new issue to ask a question.

## Issues

If you find any issue with the library, you can report it on the issue tracker <a href="https://github.com/DanielChappuis/reactphysics3d/issues" target="_blank">here</a>.
Please, do not open a new issue to ask a question. Use the <a href="https://github.com/DanielChappuis/reactphysics3d/discussions" target="_blank">Discussions</a> section for your questions.

## Author

The ReactPhysics3D library has been created and is maintained by <a href="https://github.com/DanielChappuis" target="_blank">Daniel Chappuis</a>.

## License

The ReactPhysics3D library is released under the open-source <a href="http://opensource.org/licenses/zlib" target="_blank">ZLib license</a>.

## Sponsorship

If you are using this library and want to support its development, you can sponsor it <a href="https://github.com/sponsors/DanielChappuis" target="_blank">here</a>.

## Credits

Thanks a lot to Erin Catto, Dirk Gregorius, Erwin Coumans, Pierre Terdiman and Christer Ericson for their amazing GDC presentations,
their physics engines, their books or articles and their contributions on many physics engine forums.

Thanks to all the contributors that have reported issues or have taken the time to send pull requests.

 - [Artbake Graphics](https://sketchfab.com/ismir) for the static castle 3D model used in testbed application ([CC license](https://creativecommons.org/licenses/by/4.0))

