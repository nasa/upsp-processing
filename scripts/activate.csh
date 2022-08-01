#!/bin/csh
#
# Script to set user's environment appropriately for
# running UPSP applications in this release of the
# software.
#
# Key requirements for this script that need to be
# satisfied at ALL TIMES if anything else is added:
#
# 1. Idempotent - the user should be able to source
#    this script repeatedly and the effects on their
#    environment should remain the same after repeated
#    operations (e.g., the PATH should not grow without
#    bound).
#
# 2. Relocatable - the user should be able to copy
#    the release folder around, and this script should
#    continue to work as long as the layout/content
#    of the release folder is not modified.
#

if ( `where module` == "" ) then
    echo "ERROR: module command not found. Did you source /usr/local/lib/global.cshrc?"
    exit
endif

if (! $?_UPSP_SYSTEM_PATH ) then
    if (! $?PATH ) then
        setenv _UPSP_SYSTEM_PATH ""
    else
        setenv _UPSP_SYSTEM_PATH $PATH
    endif
endif

if (! $?_UPSP_SYSTEM_LD_LIBRARY_PATH ) then
    if (! $?LD_LIBRARY_PATH ) then
        setenv _UPSP_SYSTEM_LD_LIBRARY_PATH ""
    else
        setenv _UPSP_SYSTEM_LD_LIBRARY_PATH $LD_LIBRARY_PATH
    endif
endif

if (! $?_UPSP_SYSTEM_PYTHONPATH ) then
    if (! $?PYTHONPATH ) then
        setenv _UPSP_SYSTEM_PYTHONPATH ""
    else
        setenv _UPSP_SYSTEM_PYTHONPATH $PYTHONPATH
    endif
endif


# Robust means of determining canonical path to THIS
# script's directory, both when it's sourced and when it's
# executed. Only edge case is if this file is symlink'ed,
# in which case a more thorough solution is necessary.
set CALLED=($_)
if ( "$CALLED" != "" ) then
    # called by source 
    set THIS_FN=`readlink -f $CALLED[2]`
else
    # called by direct execution of the script
    set THIS_FN=`readlink -f $0`
endif
set THIS_DIR=`dirname $THIS_FN`

# If this script is contained in a valid git repository, then
# we assume we are in development mode. Otherwise, we assume
# we are in an install.
set UPSP_BIN_DIR = $THIS_DIR/bin
set UPSP_PYTHON_DIR = $THIS_DIR/python
set UPSP_DEV_DIR = `bash -c "cd $THIS_DIR && git rev-parse --show-toplevel 2> /dev/null"`
set _UPSP_VERSION = `$THIS_DIR/version`
if ( -d "${UPSP_DEV_DIR}" ) then
    set UPSP_BIN_DIR = "$UPSP_DEV_DIR/build:$UPSP_DEV_DIR/scripts"
    set UPSP_PYTHON_DIR = "$UPSP_DEV_DIR/python"
    set _UPSP_VERSION = `bash -c "cd $THIS_DIR && git describe 2> /dev/null"`
endif

setenv _UPSP_RUNTIME_ROOT $THIS_DIR
setenv _UPSP_VERSION $_UPSP_VERSION
setenv PATH "${UPSP_BIN_DIR}:$_UPSP_SYSTEM_PATH"

# todo-mshawlec: right now, we use conda to
# manage third-party Python dependencies, and
# then the PYTHONPATH to "layer" the installed
# uPSP package on top of it. An improvement could
# be to provide the uPSP package itself as a
# conda package using the 'conda build' tooling.
source "/swbuild/upsp/miniconda3/etc/profile.d/conda.csh"
conda activate upsp

setenv PYTHONPATH "${UPSP_PYTHON_DIR}:${_UPSP_SYSTEM_PYTHONPATH}"
