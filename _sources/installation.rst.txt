============
Installation
============

The project does not currently provide pre-built binary releases. Users must
build the project from source on their local system. The project is built as
a Python package with native applications and extension modules written in C/C++.

The following instructions will help the user build and install the package into
their local Python environment as a ``pip`` package. The package build system
leverages ``scikit-build`` and CMake to compile native C/C++ elements and deploy
them alongside pure Python modules in the final package.

Prerequisites
=============

- The project is currently tested for use on Linux x86-64 target systems.
- See :ref:`dependencies:Third-Party Dependencies` for a list of third-party libraries that must be installed
  on the local system in order to build the project. They can be managed using the
  system package manager (``apt`` on Ubuntu, ``yum`` on CentOS, etc.) or with a C/C++
  package management framework---the package maintainers at NASA develop and test using
  `vcpkg <https://vcpkg.io/en/index.html>`_. If using ``vcpkg``, the dependencies will likely not be in a default system folder and
  so the following environment variables should be set to point the build system at the
  CMake toolchain file provided by your ``vcpkg`` install:
  
  .. code:: bash

     #!/bin/bash
     export SKBUILD_CONFIGURE_OPTIONS=" -DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"

- Python must be installed on the local system (3.7+), ideally inside a virtual environment.
- We require ``pip>=22.3.1``. To upgrade, you can run ``pip install --upgrade pip``.

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
