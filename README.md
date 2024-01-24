# ComplexGen Trim

## Environment Preparation

1. following [**ComplexGen**](https://github.com/guohaoxiang/ComplexGen)
2. install **[OCCT](https://dev.opencascade.org/)**
3. using vcpkg to install missing library

### Compile and build

```cmd
$ cd PATH_TO_COMPLEXGEN_TRIM
$ mkdir build
$ cd build 
$ cmake ..
```

Then you can build *ComplexGenTrim.sln* with Visual Studio. After that, you'll find *ComplexGenTrim.exe* under *PATH_TO_COMPLEXGEN_TRIM/Bin*.

### Process Data Environment setup

```cmd
$ pip install open3d
```

