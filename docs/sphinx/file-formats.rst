============
File Formats
============

The following file formats are used by the uPSP processing software.

.. _sec-video-file:

High-speed camera video formats
===============================

The uPSP software supports the following vendor-specific files:

-  CINE files (from Phantom Vision cameras)
-  MRAW/CIH files (from Photron cameras)

In particular, the software is tested primarily with monochannel,
“12-bit packed” images. Other packing strategies are supported but are
less commonly used.

.. _sec-grid-file:

Surface grid definition
=======================

The uPSP software requires an accurate definition of the wind tunnel
model wetted surfaces. It currently supports two file formats,
auto-detected based on file extension:

-  ``*.g``, ``*.x``, ``*.grid``, ``*.grd``: PLOT3D structured grid
-  ``*.tri``: Cart3D unstructured grid

The following subsections provide more detail for each grid file format.

.. _sec-plot3d:

PLOT3D file formats
-------------------

`PLOT3D <https://software.nasa.gov/software/ARC-14400-1>`_ is a computer
graphics program (written at NASA) to visualize the
grids and solutions of computational fluid dynamics.
The program defined several file formats that are now industry standard;
in particular, refer to
`PLOT3D User Manual, Chapter 8: Data File Formats <https://ntrs.nasa.gov/citations/19900013774>`_.
The uPSP software processes the following subset of PLOT3D file formats:

-  3D Grid XYZ file (``*.grd``, ``*.x``, ``*.g``) (PLOT3D User Manual,
   Section 8.2)

   -  Unformatted FORTRAN (binary, and contains leading- and trailing-
      32-bit record separators)
   -  Multi-grid
   -  Single precision
   -  No IBLANKS

-  3D Function file (``*.p3d``, ``*.f``) (PLOT3D User Manual, Section
   8.4)

   -  Unformatted FORTRAN (binary, and contains leading- and trailing-
      32-bit record separators)
   -  Multi-grid
   -  Single precision
   -  Assumes only one scalar variable per vertex (in user manual,
      ``NVAR=1``)

Note that the uPSP software does *not* ingest solution files (``*.q``),
which normally contain values for the full flow variable solution at
each grid point. Solution files are more commonly produced by CFD
solvers and are occasionally confused with function files.

.. _sec-cart3d:

Cart3D file formats
-------------------

`Cart3D <https://www.nas.nasa.gov/publications/software/docs/cart3d/pages/cart3dTriangulations.html>`_
is a high-fidelity inviscid analysis package (written at NASA) for conceptual and preliminary
aerodynamic design. The package defined several `grid file formats <https://www.nas.nasa.gov/publications/software/docs/cart3d/pages/cart3dTriangulations.html>`_
that are now industry standard. The uPSP software processes the following subset of Cart3D file formats:

-  Surface triangulations (``*.tri``) for 3D surface geometry definition
-  Annotated triangulations (``*.triq``) for scalar values defined over
   a surface triangulation

.. _sec-tgts-file:

"Targets" file (``*.tgts``)
===========================

In addition to a specification of the model surface geometry, the uPSP
software must be provided a file that specifies:

-  The 3D position and normal for visible key points on the model surface, known
   as targets

   -  These key points must be dark regions on the model surface where
      paint was either not applied to purposefully darked with a stain

-  Circular surface regions (position + diameter) where paint is
   not present or is damaged

The targets locations must be outside the model grid by a distance of
1e-5 +/- 5e-6 inches.

The file format is commonly called the “targets” file and is defined by
the DOTS application, a steady-state PSP solver application used at
NASA, but should be modified to ensure the targets are consistent with
the grid file. An example is included below.

Targets File Example:

.. code::

   #   x            y           z           normal x  normal y  normal z size  i    j   k   name
   1   67.02449799  3.17536973 -2.70057420  0.000000  0.762017 -0.647557 0.433 596  21  55  st6
   2   66.52809906  2.83113538  3.05953808  0.000000  0.678209  0.734869 0.433 596  85  50  st7
   3   79.28500366  7.62883166 -1.21192827  0.000420  0.742280 -0.670089 0.433 2261 160 26  st8
   4   79.03060150  7.57744602  1.26724449 -0.000430  0.718970  0.695040 0.433 2261 20  23  st9
   5   81.26619721  2.67079590  3.20469503  0.000000  0.641469  0.767148 0.433 716  29  25  st10
   6   95.46779633  7.70066046  1.12676893  0.000000  0.786289  0.617859 0.433 2260 17  30  st11
   7   98.17250061  2.85770474  3.03950497  0.000000  0.677413  0.735603 0.433 716  189 27  st12
   8  110.63110352  7.77465312  1.02533593  0.000000  0.826620  0.562760 0.433 2260 15  224 st13


.. _sec-steady-file:

Steady-state surface pressure file
==================================

The uPSP software requires a reference, steady-state surface pressure at
each grid point in order to compute the unsteady fluctuating pressure.
It supports the following file formats for ingesting steady-state
pressure data, auto-detected by file extension. It is also dependent on
the format used for the model grid file.

-  PLOT3D function file (``*.f``, ``*.p3d``) if the model grid is PLOT3D
   structured
-  Cart3D annotated triangulation (``*.triq``) if the model grid is
   Cart3D unstructured

See :ref:`file-formats:PLOT3D file formats` and :ref:`file-formats:Cart3D file formats`
for more details. In both formats, the scalar values
should be provided as values of the coefficient of pressure (:math:`C_p`).

.. _sec-wtd-file:

Wind Tunnel Data (``*.wtd``)
============================

During operations at the NASA Ames UPWT Complex 11-ft test section,
time-averaged values for positioning of the model in the test
section and current flow conditions are provided
by the wind tunnel data systems via a simple, human-readable
text file simply called the Wind Tunnel Data (``*.wtd``) file.

An example WTD file is shown below. Tunnel data system values are written
tab-delimited with their name in the corresponding header column.

.. code::

   # MACH     ALPHA     BETA     PHI        STRUTZ_UNC
     0.838751 -0.041887 0.011826 -90.000000 3.012192

There are several uPSP applications that make use of data from
this file:

- The wind-on external calibration step requires an initial guess of
  the model positioning in the test section
- The "intensity-to-pressure" paint calibration process requires
  the steady-state flow conditions for empirical estimates of the
  model surface temperature in cases where a per-grid-point temperature
  input file is not provided

The uPSP applications make use of the following columns from the
WTD file (which may have many more columns that are ignored by the uPSP code):

-  ``STRUTZ_UNC``: (Uncorrected) "strutz" position, or current Z-coordinate of the model strut
-  ``ALPHA``: (Corrected) Model pitch angle
-  ``BETA``: (Corrected) Model sideslip angle
-  ``PHI``: (Corrected) Model roll angle

The model angles are "corrected" by the wind tunnel data system to
account for additional bending due to deflection of the sting and
model mounting system. All coordinates and angles are consistent with definitions
published by NASA for the 11-ft test section. Technical details about the NASA
Ames 11-ft test section, including coordinate system definitions and
naming conventions, are published online by NASA (see
`Unitary Plan Wind Tunnel 11-by 11-foot TWT Test Section <https://www.nasa.gov/centers/ames/orgs/aeronautics/windtunnels/11x11-wind-tunnel.html>`_).

.. _sec-upsp-gain-file:

Unsteady PSP gain calibration
=============================

The uPSP software requires a pre-computed calibration for converting
intensity ratios from camera pixel values into physical units of
pressure.

The calibration has 6 coefficients (``a``, ``b``, ``c``, ``d``, ``e``,
``f``) that should be provided in a plain text file, example as follows:

.. code::

   a = 1.1
   b = 2.2
   c = 3.3
   d = 4.4
   e = 5.5
   f = 6.6

See :ref:`swdd:Phase 2 processing` for their usage in a polynomial
relation between pressure, temperature, and the unsteady gain value.

.. _sec-cam-cal-file:

Camera-to-tunnel calibration
============================

The camera-to-tunnel calibration file contains the intrinsic camera
parameters as well as the extrinsic camera parameters relative to the
tunnel origin. It is a JSON file with the following elements:

-  ``uPSP_cameraMatrix``: Camera Matrix formatted as ``[[f, 0, dcx], [0, f,
   dcy], [0, 0, 1]]``, where f is the focal length (in pixels), and (dcx,
   dcy) is the vector in pixel space from the image center to the
   principal point.
-  ``distCoeffs``: OpenCV 5-parameter lens distortion coefficients formatted
   as ``[k1, k2, p1, p2, k3]``
-  ``rmat``: rotation matrix from camera to tunnel
-  ``tvec``: translation vector from camera to tunnel

Optional:

-  ``sensor_resolution``: Camera sensor resolution
-  ``sensor_size``: Camera sensor physical size
-  ``Updated``: Date of last update to this file

.. _sec-input-deck:

``psp_process`` configuration (``*.inp``)
=========================================

The input deck file was designed to coordinate most of the inputs and
options needed for ``psp_process``. It is also a good reference for
which files influenced the final processed results. Descriptions of all
the variables included in the input deck are included in :numref:`input-deck`

.. code::

   @general
       test = my-test-event-name
       run = 1234
       sequence = 56
       tunnel = ames_unitary
   @vars
       dir = /nobackup/upsp/test_name
   @all
       sds = $dir/inputs/123456.wtd
       grid = $dir/inputs/test-subject.grid
       targets = $dir/inputs/test-subject.tgts
   @camera
       number = 1
       cine = $dir/inputs/12345601.cine
       calibration = $dir/inputs/cam01-to-model.json
       aedc = false
   @camera
       number = 2
       filename = $dir/inputs/12345602.mraw
     calibration = $dir/inputs/cam02-to-model.json
       aedc = false
   @options
       target_patcher = polynomial
       registration = pixel
       overlap = best_view
       filter = gaussian
       filter_size = 3
       oblique_angle = 70
       number_frames = 2000
   @output
       dir = $dir/outputs


.. _input-deck:
.. table:: ``psp_process`` input deck parameter descriptions.

   .. list-table::
      :widths: 5 10 20 5 60
      :header-rows: 1
   
      * - Section
        - Variable
        - Description
        - Required?
        - How it is used
   
      * - General
        - 
        - 
        - 
        - 
   
      * - 
        - test
        - test id number
        - yes
        - included in output HDF5 files
   
      * - 
        - run
        - run number
        - yes
        - included in output HDF5 files
   
      * - 
        - sequence
        - sequence number
        - yes
        - included in output HDF5 files
   
      * - 
        - tunnel
        - tunnel identifier
        - yes
        - used for determining which tunnel transformations and input files to expect,
          only currently support ``ames_unitary``
          
      * - Vars
        - 
        - allows variables to be set for use within the file
        - no
        - any variable can be used anywhere else in the file when preceeded with ``$``, it will
          be replaced with the value when processed
   
      * - All
        - 
        - 
        - 
        - 
   
      * - 
        - sds
        - wind tunnel data (WTD) file
        - yes
        - many variables are included in the output HDF5 files; used to determine the
          orientation of the model for calibration; used as part of converting
          camera intensity to pressure
   
      * - 
        - grid
        - grid file
        - yes
        - will be the basis of the projection from the image plane into space, data will be
          stored, when available, at each grid node
          
      * - 
        - targets
        - targets file
        - yes
        - targets used to correct the calibration for this data point; targets and
          fiducials: patched over by the target patcher
   
      * - 
        - normals
        - grid vertex normals override
        - no
        - allows for individually setting the normal direction for individual grid nodes, used as part
          of projection, useful for non-watertight structured grids
   
      * - Camera
        - 
        - 
        - 
        - need a block per camera that will be processed
   
      * - 
        - number
        - camera id number
        - yes
        - used to match cine files to the correct camera calibration, should not have duplicate camera numbers
   
      * - 
        - cine
        - cine file
        - yes
        - path to the
          camera video file
          that will be
          processed
          (deprecated;
          prefer “filename”
          key instead)
   
      * - 
        - aedc
        - aedc cine file type flag
        - no
        - aedc format is
          different than other cine file formats, so it is used to read the cine
          file; default is false
   
      * - 
        - filename
        - video file
        - yes
        - path to the
          camera video file. Supported extensions: \*.mraw, \*.cine.  For
          \*.mraw, the \*.cih header file must be a sibling file of the \*.mraw
          file with the same basename, e.g., ``video-01.mraw`` and
          ``video-01.cih``.
   
      * - 
        - calibration
        - model-to-camera external calibration file
        - yes
        - path to external
          camera calibration file (output from ``upsp-extern al-calibration``;
          calibration of camera frame relative to position of wind tunnel model
          in the first frame of the camera video file).
   
      * - Options
        - 
        - 
        - 
        - 
   
      * - 
        - target_patcher
        - type of target patching
        - no
        - decide what type of target patching is implemented, supports either ``polynomial`` or
          ``none``; default is ``none``
   
      * - 
        - registration
        - image registration type
        - no
        - decide what type of image registration to perform, supports either ``pixel`` or
          ``none``; default is ``none``
    
      * - 
        - pixel_interpolation
        - pixel-based image registration interpolation method
        - no
        - for image registration type ``pixel``, decide what type of interpolation method is
          used, supports either ``linear`` or ``nearest``; default is ``linear``

      * - 
        - overlap
        - multi-view handling
        - no
        - specify how to handle points that are visible from multiple cameras, supports either
          ``best_view`` or ``average_view``; default is ``average_view``
   
      * - 
        - filter
        - image plane filtering
        - no
        - decide what type of filtering to apply to each image prior to projection, supports
          either ``gaussian`` or ``none``; default is ``none``
   
      * - 
        - filter_size
        - size of the filter
        - yes
        - decide how large the filter will be in pixels, must be odd
   
      * - 
        - oblique_angle
        - minimum projection angle
        - no
        - minimum angle between grid surface plane and camera ray to be considered visible by
          the camera; default 70 (degrees)
   
      * - 
        - number_frames
        - number of frames to process
        - yes
        - number of camera frames to process (-1 = all frames)
   
      * - Output
        - 
        - 
        - 
        - 
   
      * - 
        - dir
        - output directory
        - yes
        - destination directory for output files
   

.. _sec-excal-file:

External camera calibration configuration parameters
====================================================

The external calibration application requires a set of configuration
parameters. These parameters are mainly static parameters related to the
tunnel, model setup in the tunnel, or hyper parameters related to the
external calibration process. This input is stored as a JSON with the
following elements:

-  ``oblique_angle``: Oblique viewing angle for target visibility checks
-  ``tunnel-cor_to_tgts_tvec``: Tunnel center of rotation to targets frame
   translation vector
-  ``tunnel-cor_to_tgts_rmat``: Tunnel center of rotation to targets frame
   rotation matrix
-  ``tunnel-cor_to_tunnel-origin_tvec``: Tunnel center of rotation to tunnel
   origin translation vector
-  ``tunnel-cor_to_tunnel-origin_rmat``: Tunnel center of rotation to tunnel
   origin rotation matrix
-  ``dot_blob_parameters``: Blob detection parameters to find sharpie
   targets
-  ``dot_pad``: Sharpie target padding distance for sub-pixel localization
-  ``kulite_pad``: Kulite target padding distance for sub-pixel localization
-  ``max_dist``: Maximum matching distance between a wind-off target
   position and a detected target
-  ``min_dist``: Minimum distance between two targets before they become too
   close and ambiguous

Optional:

-  ``Updated``: Date of last update to this file

Batch processing configuration
==============================

Datapoint index
---------------

Example:

.. code:: json

   {
     "__meta__": {
       "config_name": "<string: alias for this index file, e.g., 'default'>",
       "test_name": "<string: alias for the wind tunnel test name, e.g., 'my-test-event-name'>"
     }
     "<string: unique datapoint ID number, e.g., '123456'>": {
       "camera_tunnel_calibration_dir": "<string: directory containing camera-to-tunnel calibration files>",
       "camera_video_dir": "<string: path to directory containing camera video files>",
       "grid_file": "<string: path to 3D model grid file>",
       "kulites_files_path": "<string: path to folder containing kulite *.slow, *.fast, *.info files>",
       "normals_file": "<string (optional): path to grid normals override file>",
       "paint_calibration_file": "<string: path to uPSP unsteady gain calibration file>",
       "steady_psp_file": "<string: path to steady-state PSP PLOT3D function file>",
       "targets_file": "<string: path to targets file containing registration targets and fiducials>",
       "wtd_file": "<string: path to wind tunnel data file>"
     }
   }

-  Each input file is described in more detail in other sections of this chapter
-  In the case of ``*.mraw``-formatted camera video files, it is assumed
   that the corresponding ``*.cih`` video header file is a sibling of
   the ``*.mraw`` file in the same folder, with the same name (e.g.,
   ``12345601.mraw`` and ``12345601.cih``).

Processing pipeline configuration parameters
--------------------------------------------

The processing parameters file is a single location for the user to
specify parameter values for all applications in the processing
pipeline.

The file is structured to allow the user to specify a set of default
values as well as one or more “overlays” to customize parameter values
for individual datapoints or for sets of datapoints that share common
characteristics. Each overlay is applied to a datapoint based on whether
one or more key-value pairs in its entry in the datapoint index (see
:ref:`file-formats:Datapoint index`) matches a series of regular
expressions (specified using Python-format syntax, see
`here <https://docs.python.org/3/library/re.html#regular-expression-syntax>`__).

Example:

.. code:: json

   {
     "__meta__": {
       "name": "<string: alias for these parameter settings, e.g., 'default'>"
     },
     "processing": {
       "defaults": {
         "psp_process": {
           "cutoff_x_max": 120,
           "filter": "none",
           "filter_size": 1,
           "number_frames": 20,
           "oblique_angle": 70,
           "registration": "pixel",
           "target_patcher": "polynomial"
         },
         "external-calibration": {
           "Updated": "07/27/2021",
           "dot_blob_parameters": [
             ["filterByColor", 1],
             ["filterByArea", 1],
             ["filterByCircularity", 1],
             ["filterByInertia", 1],
             ["filterByConvexity", 1],
             ["thresholdStep", 1],
             ["minThreshold", 13],
             ["maxThreshold", 40],
             ["minRepeatability", 4],
             ["minDistBetweenBlobs", 0],
             ["blobColor", 0],
             ["minArea", 4],
             ["maxArea", 24],
             ["minCircularity", 0.72],
             ["maxCircularity", 0.95],
             ["minInertiaRatio", 0.25],
             ["maxInertiaRatio", 1.01],
             ["minConvexity", 0.94],
             ["maxConvexity", 1.01]
           ],
           "dot_pad": 4,
           "image_dims": [512, 1024, 1],
           "kulite_pad": 3,
           "max_dist": 6,
           "min_dist": 10,
           "model_length": 87.1388,
           "oblique_angle": 70,
           "sensor_resolution": [800, 1280, 1],
           "sensor_size": [0.8818898, 1.4110236],
           "sting_offset": [-195.5125, 0, 0],
           "tgts_transformation_rmat": [
             [1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]
           ],
           "tgts_transformation_tvec": [-32.8612, 0, 0],
           "tunnel-cor_to_tgts_tvec": [-195.5125, 0, 0]
         }
       },
       "test-model-geometry-2": {
         "external-calibration": {
           "model_length": 123.456
         }
       },
       "test-model-geometry-3": {
         "external-calibration": {
           "model_length": 150.456
         }
       },
       "__overlays__": [
           ["defaults", {".*": ".*"}],
           ["test-model-geometry-2", {"grid_file": ".*test-model-geometry-2.*"}]
           ["test-model-geometry-3", {"grid_file": ".*test-model-geometry-3.*"}]
       ]
     }
   }

Each child of ``"processing"`` NOT named ``__overlays__`` specifies
some/all parameter values for some/all applications in the pipeline, and
is referred to here as a “parameter set”. Each parameter set is given a
name, for example, ``"defaults"`` or ``"test-model-geometry-3"`` in the above file.
Each parameter set need not specify every available application
parameter; in the example above, ``"defaults"`` contains a full
specification, whereas ``"test-model-geometry-3"`` contains a specific value just
to override the ``"model_length"`` parameter of the ``"external-calibration"``
pipeline application. The ``"test-model-geometry-3"`` overlay is applied to all
datapoints whose ``"grid_file"`` matches the regular expression :python:`r".*test-model-geometry-2.*"`

In general, all parameter names map to corresponding parameters supplied
directly to each pipeline application when running them individually/manually
(see :ref:`swdd:Pipeline Application Design Details`).

The ``__overlays__`` section specifies how to use the parameter sets to
configure each datapoint. The user must provide a list of overlay
entries, each in the format ``[<name>, <patterns>]``. Each datapoint
matching the contents of ``<patterns>`` will use the parameter values
given by the parameter set named ``<name>``. Overlays are applied in the
order listed, so in the example above, the ``config123`` parameter set
will override any values already specified in ``defaults``. The usage of
``<patterns>`` for matching a datapoint is as follows:

-  ``<patterns>`` is a dictionary where each key ``k`` and each value
   ``v`` are regular expression strings (Python-format)
-  For each entry in ``<patterns>``, and for each input in the index
   JSON (``grid_file``, ``targets_file``, etc.)

   -  If the input key matches the regular expression given by ``k``,
      then
   -  The input value is tested against the regular expression given by
      ``v``.

-  If all tests pass for all entries in ``<patterns>``, then the
   datapoint “matches.”

Portable Batch Scheduler (PBS) job configuration parameters
-----------------------------------------------------------

Example:

.. code:: json

   {
     "__meta__": {
       "name": "<string: alias for these NAS settings, e.g., 'default'>"
     },
     "nas": {
       "__defaults__": {
         "charge_group": "<string: UNIX group id, e.g., 'g1234'>",
         "node_model": "<string: NAS node mode, e.g., 'ivy'>",
         "queue": "<string: NAS job queue name, e.g., 'normal'>",
         "number_nodes": "<integer: number of NAS nodes for processing, e.g., 10>",
       },
       "external-calibration": {
         "launcher": "parallel",
         "wall_time": "<string: wall clock time to run this step, per data point, e.g., '00:10:00'>"
       },
       "extract-first-frame": {
         "launcher": "parallel",
         "wall_time": "<string: wall clock time to run this step, per data point, e.g., '00:10:00'>"
       },
       "psp_process": {
         "launcher": "serial",
         "number_nodes": "<integer: number of NAS nodes for processing, e.g., 40>",
         "wall_time": "<string: wall clock time to run this step, per data point, e.g., '00:10:00'>"
       },
       "render-images": {
         "launcher": "parallel",
         "wall_time": "<string: wall clock time to run this step, per data point, e.g., '00:10:00'>"
       }
     }
   }

Each step in the pipeline should be assigned a full set of NAS job
parameters. A set of ``__defaults__`` will apply to all steps, and then
optional overrides can be provided in a section named for each step. The
job parameters are used by ``qsub-step`` to launch step jobs on the NAS
cluster nodes, and are tuned primarily by the number of vertices in the
grid file and the number of frames in each camera video file. See
:ref:`quick-start:Guidelines for setting NAS PBS job parameters` for more details.

Plotting configuration parameters
---------------------------------

Example:

.. code:: json

   {
     "__meta__": {
       "name": "<string: alias for these plotting settings, e.g., 'default'>"
     },
     "plotting": {
       "render-images": {
         "scalars": {
           "steady_state": {
             "display_name": "Steady State",
             "contours": [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
           },
           "rms": {
             "display_name": "delta Cp RMS",
             "contours": [0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03]
           }
         },
         "grids": {
           "config111": {
             "views": {
               "left-side": {
                 "x": 123.4, 
                 "y": 123.4,
                 "z": 123.4,
                 "psi": 90.0,
                 "theta": 0.0,
                 "width": 120.0,
                 "imagewidth": 1280
               },
             },
             "exclude_zones": [101, 120]
           }
         }
       },
       "generate-miniwall": {
         "variables": [
           {"name": "mach", "format": "%4.3f", "values": [0.8, 0.85, 0.9]},
           {"name": "alpha", "format": "%4.2f", "values": [0.0, 5.0, 10.0]},
           {"name": "beta", "format": "%4.2f", "values": [0.0, 5.0, 10.0]}
         ],
         "selectors": ["alpha", "beta"],
         "layout": {"cols": ["mach"], "rows": ["alpha", "beta"]},
         "site": {
           "page_name": "My MiniWall",
           "images_dir": "images",
           "model_config_prefix": "PREFIX"
         },
         "run_log": {
           "model_config": "111",
           "path": "my-run-log.txt"
         },
         "image_discovery": {
           "ext": "png",
           "patterns": {
             "datapoint": "[0-9]{6}",
             "panel_name": "[0-9][0-9]"
           },
           "cache": "lib/image_discovery.json"
         }
       }
     }
   }

All parameters map to corresponding elements of the ``upsp-plotting``
configuration file (see :ref:`swdd:Pipeline Application Design Details`).

Output files from ``psp_process``
=================================

Output data from ``psp_process`` consists of :math:`\Delta C_p`
measurements versus time for all nodes of the user-provided wind tunnel
model grid. The measurements are provided either in raw binary format or
bundled into an HDF5 file. In addition, metadata containing the model
grid definition, wind tunnel conditions, and camera settings are bundled
with the measurements in the HDF5 file.

.. _sec-pressure-transpose-file:

Pressure-time history raw binary outputs
----------------------------------------

The ``psp_process`` application will produce two (often large) binary
files called ``pressure`` and ``pressure_transpose``. If using the batch
processing tools (described in :ref:`file-formats:Batch processing configuration`),
these files will be available in the ``output/10_other/additional_output`` subdirectory
for each datapoint; otherwise, they will be available in the directory
supplied to ``psp_process`` using the ``--add_out_dir`` command line
option. Both files contain the same data, however, the formatting
differs to facilitate downstream use cases.

The ``pressure`` binary file is formatted as follows. For a given
datapoint, the primary output data from ``psp_process`` is the
pressure-time history, defined as a matrix :math:`P` of size
:math:`[N_f \times N_m]` where :math:`N_f` is the number of frames from
a uPSP camera (all cameras are synchronized, so this corresponds to the
number of time steps) and :math:`N_m` is the number of grid nodes. If we
let :math:`i = 1, 2, \ldots, N_f`, and :math:`j = 1, 2, \ldots, N_m`,
then element :math:`P_{i, j}` corresponds to the unsteady pressure
(:math:`\Delta C_p`) measured during the :math:`i`\ ’th time step at the
location of the :math:`j`\ ’th model grid node (the ordering of grid
nodes corresponds one-to-one with the ordering in the user-supplied
model grid definition file; see :ref:`file-formats:Surface grid definition`).
Equivalently, rows of :math:`P` correspond to snapshots of the pressure
distribution over the entire grid at each time step. The ``pressure``
binary file contains the matrix :math:`P` written to disk in row-major
format, where each value is a 32-bit, little endian, IEEE floating point
number. This corresponds to the following sequence of matrix elements:

.. math::

   P_{1,1}, P_{1,2}, ..., P_{1,N_m}, P_{2,1}, P_{2,2}, ..., P_{2,N_m}, ..., P_{N_f,1}, P_{N_f,2}, ..., P_{N_f,N_m}

Because this file can be quite large (for example, a grid with 1 million
nodes and a datapoint with 50,000 time points will have a total of 50
billion floating point values, corresponding to approximately 186 GB),
performance of file read operations can become an issue during
post-processing. It is advantageous to read contiguous values from the
file, meaning the ``pressure`` binary file is best suited for analyses
of the *entire* model grid for a *subset* of time history.

To facilitate analyses of the *entire* time history for a *subset* of
model grid nodes, a separate ``pressure_transpose`` file is also
generated. It contains the values of the *transpose* of :math:`P`
written to disk in a similar row-major format. This corresponds to the
following sequence of matrix elements:

.. math::

   P_{1,1}, P_{2,1}, ..., P_{N_f,1}, P_{1,2}, P_{2,2}, ..., P_{N_f,2}, ..., P_{1,N_m}, P_{2,N_m}, ..., P_{N_f,N_m}

In the ``pressure_transpose`` file, the time history for each individual
grid node is contiguous in memory.

The following Python code snippet demonstrates how the
``pressure_transpose`` flat file may be used to obtain the time history
for a single model grid node:

.. code:: python

   import numpy as np
   import os

   def read_pressure_transpose(filename, number_grid_nodes, node_index):
       """ Returns a 1-D numpy array containing model grid node pressure time history
       - filename: path to pressure_transpose file
       - number_grid_nodes: total number of model grid nodes
       - node_index: index of grid node (zero-based) in model grid file
       """
       filesize_bytes = os.path.getsize(filename)
       itemsize_bytes = 4  # 32-bit floating point values
       number_frames = int((filesize_bytes / itemsize_bytes) / number_grid_nodes)
       assert(number_frames * number_grid_nodes * itemsize_bytes == filesize_bytes)
       with open(filename, 'rb') as fp:
           offset = node_index * number_frames
           fp.seek(offset, 0)
           return np.fromfile(fp, dtype=np.float32, count=number_frames)

HDF5-formatted files
--------------------

HDF5-formatted files are also provided containing the pressure-time
history solution matrix and associated metadata.

Diagnostics and quality checks
------------------------------

There are a number of additional outputs that are useful for checking
behavior of the processing code and to check quality of input data
files. By default, outputs are stored in the output directory specified
in the input deck file; this can be overridden using the
``-add_out_dir`` command line option. :numref:`psp-process-quality-checks`
describes each output file in more detail.

.. _psp-process-quality-checks:
.. list-table:: ``psp_process`` output files for diagnostics and quality checks. The prefix ``<camXX>`` refers to an image from camera ``XX``.
   :widths: 20 80
   :header-rows: 1

   * - File name
     - Description
   * - ``<camXX>-8bit-raw.png``
     - first frame, scaled to 8-bit
   * - ``<camXX>-raw.exr``
     - first frame, converted to high-dynamic-range, 32-bit, OpenEXR format. High-fidelity representation of the first frame from the camera. Pixel values are equal to those in the source image — all supported source video files have less than 32 bits per pixel, so 32-bit OpenEXR provides a way to preserve fidelity across vendor-specific input video files.
   * - ``<camXX>-8bit-projected-fiducials.png``
     - first frame, scaled to 8-bit, with fiducial positions from the ``tgts`` file projected into the frame accounting for occlusion and obliqueness checks, using the input external calibration parameters
   * - ``<camXX>-8bit-fiducial-clusters.png``
     - first frame, scaled to 8-bit, with fiducials colored according to their cluster ID (only clusters with >1 fiducial are assigned colors)
   * - ``<camXX>-8bit-cluster-boundaries.png``
     - first frame, scaled to 8-bit, showing boundaries drawn around fiducial clusters. Pixels inside the drawn boundaries will be “patched” by replacing their values with a polynomial interpolant (prior to projection to the 3D grid)
   * - ``<camXX>-nodecount.png``
     - first frame, colormapped to show # grid nodes mapped to each pixel. Ideally, if no image filter is used, then there should be an approximate one-to-one map of grid nodes to pixel; otherwise, an image filter can be used to average neighboring pixels, so that pixels with no grid node will have their value averaged into a neighboring value that does map to a grid node. If there are many more grid nodes than pixels, then it may indicate the grid resolution is over-specified compared to the resolution of the camera.
