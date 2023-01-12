Third-Party Dependencies
========================

The majority of third-party dependencies are open-source and available
from public internet endpoints. :numref:`third party deps table` contains details for
each dependency including the version, license, and description of usage
by the uPSP software.

The only current commercial dependency is the Message Passing Interface
(MPI) implementation used for operation of the uPSP software on the
National Advanced Supercomputing (NAS) cluster at NASA Ames. The
dependency is via shared library linking at build time and is required
for leveraging parallelization across NAS cluster nodes. Prior to
release of uPSP source code to non-NAS environments, this dependency
will be revisited to identify proper alternatives (there are open-source
MPT/MPI implementations available, however, they are not needed in the
NAS environment).

.. _third party deps table:
.. csv-table:: Third-Party Dependencies
   :header-rows: 1

   Name,Version,License,Description of usage
   Boost_,1.68.0,`Boost License <https://www.boost.org/users/license.html>`__,"C++ routines for efficient data structure manipulation (e.g., array iterators)"
   CMake_,3.20.5,`3-clause BSD <https://gitlab.kitware.com/cmake/cmake/raw/master/Copyright.txt>`__,Build system for uPSP project
   Eigen_,3.3.9,`MPL2 <https://www.mozilla.org/en-US/MPL/2.0/>`__,"Header-only software dependency, used for high-performance linear algebra and matrix mathematics"
   gcc_,8.2,`GPLv3 <https://gcc.gnu.org/onlinedocs/gcc-8.2.0/gcc/Copying.html#Copying>`__,Compiler used to build C/C++ uPSP applications (usage is covered under GPLv3 GCC runtime exception)
   GOMP_,1.0.0,`GPLv3 <https://gcc.gnu.org/onlinedocs/libgomp/Copying.html#Copying>`__,"GNU implementation of the OpenMP specification, for parallelization of uPSP processing (usage is covered under the GPLv3 GCC runtime exception)"
   GTest_,1.11.0,`3-clause BSD <https://github.com/google/googletest/blob/master/LICENSE>`__,Unit testing for C/C++ uPSP applications as part of the uPSP build system.
   HDF5_,1.12.0,`HDF5 License <https://www.hdfgroup.org/package/hdf5/?wpdmdl=12838&refresh=5f05197e188c41594169726>`__,Some uPSP output data artifacts are encoded in the HDF5 format.
   `JSON for Modern C++`_,3.9.1,`MIT <https://github.com/nlohmann/json/blob/develop/LICENSE.MIT>`__,"Header-only software dependency, used for parsing JSON files."
   kdtree_,0.5.7,`3-clause BSD <https://github.com/jtsiomb/kdtree/blob/master/COPYING>`__,"C library for working with KD-trees, used as part of the camera-to-model data projection process."
   Matplotlib_,3.3.2,`PSF-like <https://matplotlib.org/3.3.2/users/license.html>`__,Plotting routines for Python uPSP applications.
   Numpy_,1.19.2,`3-clause BSD <https://github.com/numpy/numpy/blob/main/LICENSE.txt>`__,Scientific computing for Python uPSP applications.
   OpenCV_,4.5.2,`3-clause BSD <https://opencv.org/license/>`__,Computer vision processing (image registration and p ost-processing).
   OpenEXR_,2.5.0,`Modified BSD <https://www.openexr.com/license.html>`__,The IlmBase library (component of OpenEXR) is used for high-performance raycasting as part of the camera-to-model data projection process.
   pbrt-v3_,3,`2-clause BSD <https://github.com/mmp/pbrt-v3/blob/master/LICENSE.txt>`__,High-performance raycasting as part of the camera-to-model data projection process. Several source code files from the pbrt project are copied + modified and distributed as part of the uPSP source code.
   Python_,3.7.9,`PSF License <https://docs.python.org/3.7/license.html>`__,The Python interpreter is a necessary system dependency for running some uPSP pipeline applications. (The Python interpreter itself is not modified or redistributed as part of the release.)
   Scipy_,1.6.0,`3-clause BSD <https://github.com/scipy/scipy/blob/master/LICENSE.txt>`__,Scientific computing for Python uPSP applications.
   `SGI MPT`_,2.17,Commercial License,uPSP software built for deployment on the NASA Advanced Supercomputing (NAS) facility leverages the NAS-provided commercial implementation of the MPI 1.1 specification for scaling of parallel processing across the NAS cluster nodes. The library is linked to uPSP applications as a shared library.

.. _Boost:                 https://www.boost.org
.. _CMake:                 https://cmake.org
.. _Eigen:                 https://eigen.tuxfamily.org
.. _gcc:                   https://gcc.gnu.org
.. _GOMP:                  https://gcc.gnu.org/projects/gomp
.. _GTest:                 https://github.com/google/googletest
.. _HDF5:                  https://www.hdfgroup.org/solutions/hdf5
.. _`JSON for Modern C++`: https://github.com/nlohmann/json
.. _kdtree:                http://nuclear.mutantstargoat.com/sw/kdtree
.. _Matplotlib:            https://matplotlib.org
.. _Numpy:                 https://numpy.org
.. _OpenCV:                https://opencv.org
.. _OpenEXR:               https://www.openexr.com/index.html
.. _pbrt-v3:               https://github.com/mmp/pbrt-v3
.. _Python:                https://www.python.org
.. _Scipy:                 https://www.scipy.org
.. _`SGI MPT`:             https://www.nas.nasa.gov/hecc/support/kb/sgi-mpt_89.html
