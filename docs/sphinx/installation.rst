============
Installation
============

There are two components of the uPSP software:
- A C/C++ code base including several utility applications and the
  main processing application ``psp-process``
- A Python code base including batch processing tooling, camera calibration
  routines, and data analysis tools. This also includes some limited integration
  of C/C++ functionality via ``pybind11`` for 2D-3D raycasting routines.

To install the C/C++ code base, the user must build from source (instructions
are below).

To install the Python code base, the user can install from source OR (simpler
solution) they can pull from the binary package distribution on PyPI by running
``pip install upsp``. The Python library is compatible solely with Python 3.9
(this may be updated soon in an upcoming release).

Our current strategy at NASA is to build from source for our supercomputing cluster,
and then install from PyPI for local development and data analysis.

Prerequisites
=============

- The C/C++ project is currently tested for use on Linux x86-64 target systems.
- Python 3.9 must be installed on the local system, ideally inside a virtual environment.
- We require ``pip>=22.3.1``. To upgrade, you can run ``pip install --upgrade pip``.
- Several CMake options can be used to control what is built.
  - ``UPSP_BUILD_APPLICATIONS``: build core applications
  - ``UPSP_BUILD_PYBIND11``: build Python C/C++ extensions (requires ``pybind11``)
  - ``UPSP_BUILD_TESTING``: build C/C++ unit tests (requires ``gtest``)
- See :ref:`dependencies:Third-Party Dependencies` for a list of third-party libraries that must be installed
  on the local system in order to build the project.
  - If building only the Python bindings, then only ``imath`` is required.
  - At NASA, we develop and test using `vcpkg <https://vcpkg.io/en/index.html>`_. If using
    ``vcpkg``, the dependencies will likely not be in a default system folder and
    so the following environment variables should be set to point the build system at the
    CMake toolchain file provided by your ``vcpkg`` install:

    .. code:: bash

      #!/bin/bash
      CMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      CMAKE_OPTS=()
      CMAKE_OPTS+=("-DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE")
      CMAKE_OPTS+=("-DUPSP_BUILD_APPLICATIONS=ON")
      CMAKE_OPTS+=("-DUPSP_BUILD_PYBIND11=ON")
      CMAKE_OPTS+=("-DUPSP_BUILD_TESTING=ON")
      export SKBUILD_CONFIGURE_OPTIONS="${CMAKE_OPTS[@]}"


Obtaining source code
=====================

You can clone the repository from (TODO), or download the latest source code archive from (TODO).

Build and install
=================

Navigate to the source code folder tree and run

.. code:: bash

   pip install -v .

This will build the ``upsp`` package and install it in your Python environment. Several executable
entrypoints will be available in your local Python environment ``bin`` folder after install.

To check successful install, you can run ``psp_process -h`` to display its application usage, and run
``python -c "import upsp.raycast; import pydoc; print(pydoc.render_doc(upsp.raycast.BVH))"`` to display
the Python API usage statement for one of the ``upsp`` C/C++ Python extension modules.

Unit Tests
==========

Unit tests for C/C++ modules are written with Google Test and are built as part of the packaging process.

- After running ``pip install``, you should have a local folder called
  ``_skbuild`` that caches the results of the build, for example,
  ``_skbuild/linux-x86_64-3.9/cmake-build``
- Navigate to ``cpp/test`` and then run ``../../_skbuild/linux-x86_64-3.9/cmake-build/run_tests``
  (Some unit tests rely on relative paths to test data located in ``cpp/test``).
