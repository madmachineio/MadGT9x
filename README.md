# MadGT9x
Touch screen gt9x device driver

# Build
1. export PATH=/zephyr-sdk-path/arm-zephyr-eabi/bin/:$PATH
2. git clone --recurse-submodules  https://github.com/madmachineio/MadGT9x.git
3. cd ./MadGT9x/
4. mkdir build
5. cd ./build
6. cmake ../
7. make

# Result
After make, MadGt9x lib and header file will be put in MadGT9x/../externlib, eg:
```
#tree ../extrenlib/
../extrenlib/
├── inc
│   └── MadGt9x.h
└── lib
    └── libmadgt9x.a
```