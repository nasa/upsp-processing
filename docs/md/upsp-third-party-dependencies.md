---
title: Unsteady Pressure-Sensitive Paint (uPSP) Data Processing Software Third Party Software Analysis
tablenos-number-by-section: true
xnos-capitalise: true
xnos-warning-level: 1
author:
- Marc A. Shaw-Lecerf
- Jessie Powell
...

# Introduction

The following document summarizes the usage of third-party software libraries
by the Unsteady Pressure-Sensitive Paint (uPSP) data processing software.

# Summary of third-party dependencies

The majority of third-party dependencies are open-source and available from
public internet endpoints. +@tbl:third-party contains details for each dependency including the version, license, and
description of usage by the uPSP software.

The only current commercial dependency is the Message Passing Interface (MPI) implementation used for operation of the uPSP software on the National Advanced Supercomputing (NAS) cluster at NASA Ames. The dependency is via shared library linking at build time and is required for leveraging parallelization across NAS cluster nodes. Prior to release of uPSP source code to non-NAS environments, this dependency will be revisited to identify proper alternatives
(there are open-source MPT/MPI implementations available, however, they are not needed in the NAS environment).

Table: uPSP third-party dependencies overview {#tbl:third-party}

 Name                      Version        License         Description of usage
------                    --------- --------------------  ----------------------------
[Boost][B1]               1.68.0    [Boost License][B1R]  C++ routines for efficient data structure manipulation (e.g., array iterators)
[CMake][C1]               3.20.5    [3-clause BSD][C1R]   Build system for uPSP project
[Eigen][E3]               3.3.9     [MPL2][E3R]           Header-only software dependency, used for high-performance linear algebra and matrix mathematics
[gcc][G1]                 8.2       [GPLv3][G1R]          Compiler used to build C/C++ uPSP applications (usage is covered under GPLv3 GCC runtime exception)
[GOMP][G2]                1.0.0     [GPLv3][G2R]          GNU implementation of the OpenMP specification, for parallelization of uPSP processing (usage is covered under the GPLv3 GCC runtime exception)
[GTest][G3]               1.11.0    [3-clause BSD][G3R]   Unit testing for C/C++ uPSP applications as part of the uPSP build system.
[HDF5][H1]                1.12.0    [HDF5 License][H1R]   Some uPSP output data artifacts are encoded in the HDF5 format.
[JSON for Modern C++][J1] 3.9.1     [MIT][J1R]            Header-only software dependency, used for parsing JSON files.
[kdtree][K1]              0.5.7     [3-clause BSD][K1R]   C library for working with KD-trees, used as part of the camera-to-model data projection process.
[Matplotlib][M1]          3.3.2     [PSF-like][M1R]       Plotting routines for Python uPSP applications.
[Numpy][N1]               1.19.2    [3-clause BSD][N1R]   Scientific computing for Python uPSP applications.
[OpenCV][O1]              4.5.2     [3-clause BSD][O1R]   Computer vision processing (image registration and post-processing).
[OpenEXR][O2]             2.5.0     [Modified BSD][O2R]   The IlmBase library (component of OpenEXR) is used for high-performance raycasting as part of the camera-to-model data projection process.
[pbrt-v3][P1]             3.0       [2-clause BSD][P1R]   High-performance raycasting as part of the camera-to-model data projection process. Several source code files from the pbrt project are copied + modified and distributed as part of the uPSP source code.
[Python][P2]              3.7.9     [PSF License][P2R]    The Python interpreter is a necessary system dependency for running some uPSP pipeline applications. (The Python interpreter itself is not modified or redistributed as part of the release.)
[Scipy][S1]               1.6.0     [3-clause BSD][S1R]   Scientific computing for Python uPSP applications.
[SGI MPT][S2]             2.17      Commercial License    uPSP software built for deployment on the NASA Advanced Supercomputing (NAS) facility leverages the NAS-provided commercial implementation of the MPI 1.1 specification for scaling of parallel processing across the NAS cluster nodes. The library is linked to uPSP applications as a shared library.


[B1]: <https://www.boost.org/>
[C1]: <https://cmake.org/>
[E3]: <https://eigen.tuxfamily.org>
[G1]: <https://gcc.gnu.org>
[G2]: <https://gcc.gnu.org/projects/gomp/>
[G3]: <https://github.com/google/googletest>
[H1]: <https://www.hdfgroup.org/solutions/hdf5/>
[J1]: <https://github.com/nlohmann/json>
[K1]: <http://nuclear.mutantstargoat.com/sw/kdtree/>
[M1]: <https://matplotlib.org/>
[N1]: <https://numpy.org/>
[O1]: <https://opencv.org/>
[O2]: <https://www.openexr.com/index.html>
[P1]: <https://github.com/mmp/pbrt-v3>
[P2]: <https://www.python.org/>
[S1]: <https://www.scipy.org/>
[S2]: <https://www.nas.nasa.gov/hecc/support/kb/sgi-mpt_89.html>

[B1R]: <https://www.boost.org/users/license.html>
[C1R]: <https://gitlab.kitware.com/cmake/cmake/raw/master/Copyright.txt>
[E3R]: <https://www.mozilla.org/en-US/MPL/2.0/>
[G1R]: <https://gcc.gnu.org/onlinedocs/gcc-8.2.0/gcc/Copying.html#Copying>
[G2R]: <https://gcc.gnu.org/onlinedocs/libgomp/Copying.html#Copying>
[G3R]: <https://github.com/google/googletest/blob/master/LICENSE>
[H1R]: <https://www.hdfgroup.org/package/hdf5/?wpdmdl=12838&refresh=5f05197e188c41594169726>
[J1R]: <https://github.com/nlohmann/json/blob/develop/LICENSE.MIT>
[K1R]: <https://github.com/jtsiomb/kdtree/blob/master/COPYING>
[M1R]: <https://matplotlib.org/3.3.2/users/license.html>
[N1R]: <https://github.com/numpy/numpy/blob/main/LICENSE.txt>
[O1R]: <https://opencv.org/license/>
[O2R]: <https://www.openexr.com/license.html>
[P1R]: <https://github.com/mmp/pbrt-v3/blob/master/LICENSE.txt>
[P2R]: <https://docs.python.org/3.7/license.html>
[S1R]: <https://github.com/scipy/scipy/blob/master/LICENSE.txt>
