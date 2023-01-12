===========
Quick Start
===========

In general, there are two ways to run the uPSP processing applications:

-  :ref:`Processing one test condition <sec-single-processing>`: Useful for initial checkout of raw
   data from a wind tunnel test setup. The user manually runs each step
   in the pipeline.
-  :ref:`Batch processing multiple test conditions <sec-batch-processing>`: Useful after initial
   checkout. A framework auto-generates scripts that will invoke each
   pipeline step for each wind tunnel test condition. The user controls
   the pipeline via top-level “launcher” scripts.

.. _sec-single-processing:

Producing surface pressure time history for one test condition
==============================================================

The following instructions allow a user to process a single test
conditions with the uPSP software. The steps will use bracketed
shorthand like ``<name>`` to refer to the path to input files on
disk (for more details concerning file formats, see :doc:`file-formats`).

#. Acquire the following input files:

   -  ``<video-file>``: High-speed video of your test subject (:ref:`sec-video-file`)
   -  ``<grid-file>``: Test subject 3D model (:ref:`sec-grid-file`)
   -  ``<tgts-file>``: Registration targets and fiducials (:ref:`sec-tgts-file`)
   -  ``<steady-file>``: Steady-state surface pressure (:ref:`sec-steady-file`)
   -  ``<wtd-file>``: Time-averaged wind tunnel conditions (:ref:`sec-wtd-file`)
   -  ``<paint-file>``: Unsteady gain paint calibration coefficients (:ref:`sec-upsp-gain-file`)
   -  ``<cal-file>``: Camera calibration parameters (one file per camera) (:ref:`sec-cam-cal-file`)

#. Create the following processing configuration files:

   -  ``<excal-cfg-file>``: Configuration for ``upsp-external-calibration`` (:ref:`sec-excal-file`)
   -  ``<inp-file>``: Input deck for ``psp_process`` (:ref:`sec-input-deck`)

#. Run ``upsp-extract-frames`` to extract the first frame from the camera video file as a PNG.

   .. code:: bash

      #!/bin/bash
      upsp-extract-frames \
        -input=<video-file> \
        -output=first-frame.png \
        -start=1 \
        -count=1

   Run ``upsp-extract-frames -h`` for more usage details.

   The output file ``first-frame.00001.png`` contains the first frame
   from the input ``<video-file>``, scaled to 8-bit depth.

#. Run ``upsp-external-calibration`` to compute the external camera
   calibration relative to the model position in the first frame (this
   step accounts for “wind-on” deflection of the model position that
   is not accounted for in the time-averaged model position reported by
   the wind tunnel data systems):

   .. code:: bash

      #!/bin/bash
      upsp-external-calibration \
        --tgts <tgts-file> \
        --grd <grid-file> \
        --wtd <wtd-file> \
        --cfg <excal-cfg-file> \
        --cal_dir <cal-dir> \
        --out_dir . \
        --img first-frame.00001.png

   ``<cal-dir>`` refers to the name of the directory containing
   ``<cal-file>``. Run ``upsp-external-calibration -h`` for more usage
   details.

   The output file ``cam01-to-model.json`` contains the external
   calibration parameters that will be fed to ``psp_proces`` in the next
   step.

#. Run ``psp_process`` to produce a (usually quite large) time history of
   the surface pressure at each point on the model grid. This can take
   significant time and memory if run on a personal computer; we recommend
   instead that the application be run in parallel on a compute cluster.
   The application is best run so that one MPI rank runs on each compute node,
   and then thread-level parallelism is leveraged by each MPI rank to scale
   across cores on a given node.

   An example PBS job script for running ``psp_process`` on the Pleaides cluster at the
   NASA Advanced Supercomputing (NAS) Division is shown below. For more details
   about PBS syntax and uPSP-specific rules of thumb for sizing the job allocation,
   see :ref:`sec-nas-parameters`.

   .. code:: bash

      #!/bin/bash
      #PBS -q normal
      #PBS -l select=40:model=ivy
      #PBS -l walltime=00:20:00
      export MPI_DSM_DISTRIBUTE=off
      export OMP_STACKSIZE=250M
      export OMP_NUM_THREADS=16
      source /usr/local/lib/global.profile
      module purge
      module load mpi-hpe/mpt.2.25
      mpiexec psp_process \
        -input_file=<inp-file> \
        -h5_out=<out-dir>/output.h5 \
        -paint_cal=<paint-file> \
        -steady_p3d=<steady-file>

   ``<out-dir>`` refers to the value of the ``@output/dir`` variable
   specified in the ``<inp-file>``. Run ``psp_process -h`` for more
   usage details.
   
   The output ``pressure_transpose`` file contains the surface pressure
   time history for each node on the model grid (see :ref:`sec-pressure-transpose-file`).
   Several diagnostic images are printed to verify the external calibration and (optional)
   fiducial patches align well with the position of the model in the
   first video frame.

#. (Optional) post-processing steps

   -  Run ``add_field`` to add the ``pressure_transpose`` data into the
      HDF5 file produced by ``psp_process``. For some of its command
      line arguments, ``add_field`` must be provided with the number of
      vertices in the 3D model and the number of frames that were
      processed. Example usage below shows how to obtain these values
      from inspecting files output by ``psp_process`` in the BASH script
      language. ``<out-dir>`` should be replaced with the same directory
      as ``<out-dir>`` in the previous step (the directory containing
      outputs from ``psp_process``).

      .. code:: bash

         #!/bin/bash

         # Set output_dir to the folder containing outputs from `psp_process`
         # in previous step.
         output_dir=<out-dir>
         trans_h5_file=$output_dir/output.h5

         # Inspect the 'X' file, which is a flat binary dump of the
         # X-coordinates of the input wind tunnel model grid vertices.
         # The number of coordinates in the file gives the size of the model.
         data_item_size=4 # coordinates stored as 4-byte float's
         model_size="$(expr $(stat --printf="%s" $output_dir/X) '/' $data_item_size)"
         trans_flat_file="$(find $output_dir -type f -name 'pressure_transpose')"
         trans_flat_file_size="$(stat --printf="%s" $trans_flat_file)"
         trans_flat_file_number_frames="$(expr $trans_flat_file_size '/' '(' $model_size '*' $data_item_size ')')"

         echo ">>> number of model nodes: $model_size"
         echo ">>> data item size: $data_item_size"
         echo ">>> time history flat file: '$trans_flat_file'"
         echo ">>> time history flat file size: $trans_flat_file_size"
         echo ">>> time history flat file number of frames: $trans_flat_file_number_frames"
         echo ">>> adding flat-file data to '$trans_h5_file'"

         ex="add_field $trans_h5_file frames $trans_flat_file $trans_flat_file_number_frames"
         echo ">>> running: '$ex'"
         t_start="$(date +%s.%N)"
         echo ">>> started at: $(date)"
         $ex
         t_end="$(date +%s.%N)"
         echo ">>> elapsed time: $(python -c "print('%4.1f' % ($t_end - $t_start))") seconds"

         echo ">>> run-add-field DONE."

.. _sec-batch-processing:

Batch processing multiple test conditions
=========================================

The following instructions allow a user to batch process one or more
test conditions from a wind tunel test with the uPSP software.

Batch processing is configured by ``upsp-make-processing-tree``, a tool
that auto-generates a file tree and associated command-line scripts that
the user can then run to execute each step in the uPSP pipeline for one
or more datapoints. The configuration process is illustrated in
:numref:`flowchart-batch-processing` and consists of the following steps:

#. The user locates raw data files from a wind tunnel test on disk
#. The user prepares four Javascript Object Notation (JSON) configuration files:

   -  A **datapoint index**, listing the path to each raw input file for
      each datapoint

      -  This often consists of writing test-specific scripts/tools to
         grok each input file on disk

   -  A **processing parameters file**, containing parameter settings
      for each step in the pipeline
   -  A **PBS job parameters file**, containing PBS scheduler
      settings (group ID, reservation wall time, number of nodes, etc.)
   -  A **plotting parameters file**, containing parameters for plotting
      steps in the pipeline

#. The user runs ``upsp-make-processing-tree`` and provides it with each
   configuration file. The script will autogenerate a file tree on disk
   to store all artifacts for batch processing

Once the processing tree is generated and saved to disk, the user
can navigate to the ``03_launchers`` subfolder and trigger each step
in the pipeline as follows:

#. Each step in the pipeline is launched using a script named
   ``step+<step-name><+optional-subtask-name>``.

   -  They should be run in the order given here (some steps use outputs
      from previous steps):

      1. ``step+extract-first-frame``: extract the first frame from each
         camera video file.
      2. ``step+external-calibration``: run the wind-on, first-frame
         external calibration for each camera.
      3. ``step+psp_process+psp-process``: run ``psp_process`` -
         image-to-grid projection and calibration to units of pressure.
      4. ``step+psp_process+add-field``: post-process ``psp_process``
         outputs; add largest pressure-time history dataset into the
         HDF5 output file.

   -  Each step launcher script can be invoked as follows:

      -  ``./<step-launcher-script> <datapoint-id-1> <datapoint-id-2> ...``
         to process a specific subset of datapoints. By default, all
         datapoints are processed.
      -  ``./qsub-step <step-launcher-script> <datapoint-id-1> <datapoint-id-2> ...``
         to launch the step on the cluster as one or more jobs (uses
         ``qsub``). The jobs can then be monitored using
         ``qstat -u $USER``. The jobs reservations are configured using
         the PBS job parameters supplied in the PBS job parameters JSON file.

#. Once complete, data products for each datapoint will be available
   under ``04_products/00_data/<step-name>/<datapoint-id>``.

The JSON file format was chosen for batch processing configuration files
due to its ubiquitous usage in industry and broad
cross-platform/cross-language support. Users should be familiar with
plain-text editing of JSON files and can reference the official JSON
syntax `here <https://www.json.org/json-en.html>`__.

.. _flowchart-batch-processing:
.. figure:: _static/flowchart-batch-processing.png
   :alt: uPSP NAS batch processing flowchart.
   :name: fig:flowchart
   :width: 100.0%

   uPSP NAS batch processing flowchart.

.. _sec-nas-parameters:

Guidelines for setting NAS PBS job parameters
=============================================

For complete details and tutorials, see the HECC wiki, `“Running Jobs with
PBS” <https://www.nas.nasa.gov/hecc/support/kb/running-jobs-with-pbs-121/>`__.

Specific to the current implementation of the uPSP processing code, the following
is rationale for practical "rules of thumb" for scaling the size of the PBS job to
your input data size. Trial-and-error may be required to define these parameters
correctly after initial best-guesses.

Given the following variable definitions:

- :math:`N_R``: Number of MPI ranks
- :math:`N_C`: Number of cameras
- :math:`F_R`: Camera resolution (pixels)
- :math:`N_M`: Number of 3D model grid nodes
- :math:`N_T`: Number of time samples

Then, tl;dr a rule-of-thumb is to ensure each MPI rank has access to at least
:math:`M_T = M_C + M_1 + M_2` bytes of local memory, where

- :math:`M_C = O\left(K_C (N_T N_C F_R)/N_R\right)` accounts for storage of camera frames
- :math:`M_1 = O\left(K_1 (N_T N_M)/N_R\right)` accounts for storage of the 3D-projected data (pixel counts)
- :math:`M_2 = O\left(K_2 (N_T N_M)/N_R\right)` accounts for storage of the calibrated, 3D-projected data (physical pressure units)

and :math:`K_C = 2`, :math:`K_1 = 8`, and :math:`K_2 = 8` are reasonable constant "fudge factors" accounting for
variability in camera bit depth and intermediate storage of elements of the solution in memory.

A practical application of this rule of thumb is as follows:

- At NASA, for one example test condition, we collected 60K frames from 4x cameras, each approximately 1MP resolution
- The wind tunnel 3D model had approximately 1M vertices
- So:

  - :math:`M_C \approx 2 \cdot 60K \cdot 4 \cdot 1M / N_R \approx 5E11 / N_R`
  - :math:`M_1 \approx 8 \cdot 60K \cdot 1M / N_R \approx 5E11 / N_R`
  - :math:`M_2 \approx 8 \cdot 60K \cdot 1M / N_R \approx 5E11 / N_R`
  - :math:`M_T \approx 1.5E12 / N_R` (bytes)

- We can use 40 MPI ranks, one per compute node, so the memory requirement per
  compute node is approximately 1.5E12 / 40 = 3.75E10 bytes, or 37.5GB. The NAS
  Ivy nodes each have `64GB of memory available <https://www.nas.nasa.gov/hecc/support/kb/preparing-to-run-on-pleiades-ivy-bridge-nodes_446.html>`_,
  so we can fit our job into a PBS session with 40 Ivy nodes and 40 MPI ranks.
  From practical experience, we know this job takes less than 10 minutes of wall clock time,
  so we can use the following PBS directive to allocate the job:

  .. code:: bash

    #PBS -lselect=40:model=ivy:walltime:00:10:00


Rationale for the rule of thumb is based on complexity analysis of the current algorithm implementation,
described in more detail in :doc:`swdd`.
