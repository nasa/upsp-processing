#!/bin/sh
#
# Script to set user's environment appropriately for
# running UPSP applications.
#
# Key requirements for this script that need to be
# satisfied AT ALL TIMES if anything else is added:
#
# 1. Idempotent - the user should be able to source
#    this script repeatedly and the effects on their
#    environment should remain the same after repeated
#    operations (e.g., the PATH should not grow without
#    bound).
#
# 2. Relocatable - this script should depend only on
#    paths relative to **it's** location, not on the
#    current working directory of the user.
#
# 3. "Development Mode" - developers should be able
#    to source this script wherever it resides in their
#    local working directory, and it should be able to
#    set up their environment to use their local build
#    of the software.

if ! command -v  module &> /dev/null
then
    echo "ERROR: module command not found. Did you source /usr/local/lib/global.profile?"
fi

if [ -z ${_UPSP_SYSTEM_PATH+x} ]; then
    export _UPSP_SYSTEM_PATH=$PATH
    export _UPSP_SYSTEM_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
    export _UPSP_SYSTEM_PYTHONPATH=$PYTHONPATH
fi

# Robust means of determining canonical path to THIS
# script's directory, both when it's sourced and when it's
# executed. Only edge case is if this file is symlink'ed,
# in which case a more thorough solution is necessary.
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# If this script is contained in a valid git repository, then
# we assume we are in development mode. Otherwise, we assume
# we are in an install.
UPSP_BIN_DIR=$THIS_DIR/bin
UPSP_PYTHON_DIR=$THIS_DIR/python
UPSP_DEV_DIR="$(cd $THIS_DIR && git rev-parse --show-toplevel 2> /dev/null)"
_UPSP_VERSION="$(cd $THIS_DIR && ./version)"
_UPSP_RUNTIME_ROOT=$THIS_DIR
if [ -d "${UPSP_DEV_DIR}" ]; then
    UPSP_BIN_DIR=$UPSP_DEV_DIR/build:$UPSP_DEV_DIR/scripts
    UPSP_PYTHON_DIR=$UPSP_DEV_DIR/python
    _UPSP_VERSION="$(cd $THIS_DIR && git describe)"
fi
export _UPSP_VERSION
export _UPSP_RUNTIME_ROOT

export PATH=$UPSP_BIN_DIR:$_UPSP_SYSTEM_PATH

# todo-mshawlec: right now, we use conda to
# manage third-party Python dependencies, and
# then the PYTHONPATH to "layer" the installed
# uPSP package on top of it. An improvement could
# be to provide the uPSP package itself as a
# conda package using the 'conda build' tooling.
__conda_setup="$('/swbuild/upsp/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/swbuild/upsp/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/swbuild/upsp/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/swbuild/upsp/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup
conda activate upsp
export PYTHONPATH=$UPSP_PYTHON_DIR:$_UPSP_SYSTEM_PYTHONPATH
