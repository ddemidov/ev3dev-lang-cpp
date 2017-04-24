# C++ language bindings for ev3dev

[![Build Status](https://travis-ci.org/ddemidov/ev3dev-lang-cpp.svg?branch=master)](https://travis-ci.org/ddemidov/ev3dev-lang-cpp)

## Compiling

* EV3:
```
mkdir build
cd build
cmake .. -DEV3DEV_PLATFORM=EV3
make
```

* BrickPi:
```
mkdir build
cd build
cmake .. -DEV3DEV_PLATFORM=RPI
make
```

You have several options for compiling.

## Cross-compiling

You can use a cross-compiling toolchain to create ARM compatible code. Note:
You need a Linux toolchain, not a "bare-metal" toolchain. If it does not have
"linux" in the name, it probably won't work.

Pros: Fastest option. Works on Windows and Mac without a virtual machine.

Cons: Only includes standard libraries - no Debian `-dev` packages.

### Windows

[MentorGraphics toolchain](http://sourcery.mentor.com/public/gnu_toolchain/arm-none-linux-gnueabi/arm-2014.05-29-arm-none-linux-gnueabi.exe) (formerly known as CodeSourcery).

### Windows 10

Windows 10 supports the Ubuntu Subsystem for Windows which allows us to install and execute the entire compiler toolchain. The steps required to compile native-mode EV3 applications on Windows 10 is as follows:

1. Be sure that your Windows 10 installation has Ubuntu Subsystem for Windows installed. To install it, go to Control Panel --> Programs and Features --> Turn Windows Feature On/Off and check the box next to Windows Subsystem for Linux (Beta).

2. Fire up the bash shell by pressing Start Key and typing Bash. This will open up Bash on Ubuntu on Windows.

3. Install the ARM compiler by typing `sudo apt-get install arm-linux-gnueabi-gcc`.

4. You need to make some changes to the top-level `CMakeLists.txt` file. First of all, set the C/C++ compilers with

```
set(CMAKE_CC_COMPILER "arm-linux-gnueabi-gcc")
set(CMAKE_CC_COMPILER "arm-linux-gnueabi-g++")
```

5. You also need the following define, without which the nanosleep-using demos won't compile:

```
add_definitions(-D_GLIBCXX_USE_NANOSLEEP)
```

6. Now compile your programs and the generated binaries will be ready for EV3. Copy them to the EV3 brick, but there's one more step!

7. Be sure to `chmod u+x myprogram` for every copied program before running the program, otherwise you'll get an `Access Denied` in SSH or some really weird error if executing from the brick.

### Mac

[Carlson-Minot toolchain](http://www.carlson-minot.com/available-arm-gnu-linux-g-lite-builds-for-mac-os-x/mac-os-x-arm-gnu-linux-g-lite-201405-29-toolchain)


## Brickstrap

Brickstrap uses QEMU to create a virtual environment (not exactly a virtual
machine) that can run the same ARM compatible code on a different type of
computer.

Pros: Faster than running on the EV3 itself. Can install all Debian `-dev`
packages using `apt-get`.

Cons: Slower than cross-compiler. Requires Linux (Ubuntu).

See [this page](https://github.com/ev3dev/ev3dev/wiki/Using-brickstrap-to-cross-compile-and-debug) for instructions.


## On the Brick

It is possible to compile programs on the EV3 brick itself.

Pros: Easy to setup.

Cons: Really slow.

Just run `sudo apt-get install build-essential` on the EV3 and you will have
everything you need.
