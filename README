This is Han Gu RNIC Simulator implemented in gem5

The gem5 website can be found at http://www.gem5.org

The gem5 version we based on is Version 20.1.0.2

Han Gu RNIC related code is founded in src/dev/rdma/, including Han Gu RNIC and 
its attached driver. The test code is located in tests/test-progs/hangu-rnic/.

# The structure of the code.

The basic source release includes these subdirectories:
   - configs: example simulation configuration scripts
   - ext: less-common external packages needed to build gem5
   - src: source code of the gem5 simulator
   - system: source for some optional system software for simulated systems
   - tests: regression tests
   - util: useful utility programs and files
   - scripts: useful scripts for RNIC build and execution

This is the V1.5 version of our implemented RNIC.

# How to Run the code?

The simulator is tested under Ubuntu 18.04.6 LTS (GNU/Linux 4.15.0-193-generic x86_64).

To build the simulator, one need to execute the scripts in scripts/ directory.


1. To begin with, you need to run ./build.sh to setup the environment:

```
./build.sh
```

2. Run analysis.py to start build & run csRNA:

```
python3 analysis.py

```


In this version, we implement the followinng features based on RNIC-V1.1: 
    1. out of order execution for qpc request in one qpc request channel, with 
    three channels in total: two for sender side, and one for receiver side.
