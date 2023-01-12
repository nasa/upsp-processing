============
Applications
============

The following sections provide an overview of the applications
distributed as part of the `upsp` software package.

Running each application with the ``-h`` option will also provide
brief usage instructions to ``stdout``.

``upsp-extract-frames``
-----------------------

The ``upsp-extract-frames`` application is a utility to dump high-speed
camera video frames to “friendlier” formats. Video frames can be output
to common image formats (e.g., jpg, png) or video formats (e.g., avi,
mp4).

``upsp-external-calibration``
-----------------------------

The ``upsp-external-calibration`` application locates model targets in
the first frame, then finds the position and orientation of the wind
tunnel model relative to the camera using photogrammetry techniques.

``psp_process``
---------------

The majority of “computational heavy lifting” is performed by the
``psp_process`` application. Conceptually, the processing is split into
three steps:

-  **Initialization**: Video files are opened; first-frame camera
   calibration parameters are loaded from file; fiducial patching is
   initialized (see +@sec:fiducial-patching); diagnostic images for each
   camera are saved; first-frame image-to-grid projections are
   initialized
-  **Phase 1**: For each time step, pixel (“intensity”) data from each
   camera frame is projected to its corresponding vertex of the model
   grid. Intensity data from multiple cameras are combined.
-  **Phase 2**: The intensity time series at each grid vertex is
   detrended and converted to a surface pressure measurement, using the
   provided unsteady gain calibration and the reference steady-state
   pressure.

For more detail on the algorithms used in each step, please see the uPSP
SwDD.

.. _`sec:fiducial-patching`:

Fiducial Patching
~~~~~~~~~~~~~~~~~

In many cases, wind tunnel test models will have surface area not
covered by pressure-sensitive paint. Particularly troublesome cases are
small regions within larger painted areas, referred to as “fiducials.”
Example fiducials could be:

-  masking tape to protect transducer heads from paint spray (often
   circular regions around a transducer head)
-  unpainted points on the model used for wind-on external calibration
   by ``upsp-external-calibration``
-  fasteners used to add/remove portions of test model between runs
-  oil stains from lubrication of test model articles

While larger regions can be excluded manually from downstream analysis,
these smaller areas are much harder to manually exclude. In this case,
``psp_process`` can automatically “patch” over known fiducial points on
the model if they are provided in the input ``tgts`` file. The position
and diameter of the fiducial must be supplied; a patch is applied in the
image plane prior to projecting to the model grid, where the patched
pixels are replaced by a 3rd-order, 2D-interpolation of the patch
boundary pixels. For fiducials that are closely-spaced such that their
patches would overlap, one larger patch is applied to the set of
fiducials (to cover use cases such as a shock array of closely-spaced
transducer heads on the model in a streamwise line). Diagnostic images
written out by ``psp_process`` include an example of the patching output
for the first frame from each video file, and can be used to manually
tune the patch algorithm parameters for best performance on a
per-wind-tunnel-test basis.
