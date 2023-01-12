============
Known Issues
============

-  Processing not robust to cases where targets defined in the
   ``*.tgts`` file are under the surface of the model grid

   -  Some targets can have 3D locations that are defined such that they
      lie under the surface of the wind tunnel model grid. Thus, these
      targets will be rejected in phase 0 processing and will not be
      used as part of the registration process.
   -  The current workaround is to perform preprocessing of the
      ``*.tgts`` file to ensure it is consistent with the model grid
      file (for instance, the locations can be offset in the direction
      of the nearest model grid surface normal by a small distance until
      they lie outside the grid surface)

-  Image registration can cause poor performance when wind tunnel model
   has a significant amount of motion

   -  The pixel-to-grid projection in Phase 0 is computed based on the
      first camera frames, and is then re-used for all camera frames;
      subsequent frames are “warped” to align with the first camera
      frame prior to projection, but tests for occlusion and obliqueness
      of model grid nodes are left unmodified
   -  However, at the edges of the model visible to a given camera, when
      the model is moving significantly, parts of the model may move in-
      and out- of view of the camera
   -  These model edges may then have degraded uPSP measurement accuracy
      when the model has lots of motion (e.g., at high Mach number).
