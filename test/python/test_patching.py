import unittest
import numpy as np
import cv2
import os
import copy

import file_structure
from upsp.cam_cal_utils import (
    camera_tunnel_calibrate,
    parsers,
    photogrammetry,
    visibility,
)
from upsp.intensity_mapping import patching

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class PatchingTestCase(unittest.TestCase):
    """
    Tests for patching
    """
    
    @classmethod
    def setUpClass(cls):       
        # Read tgts file
        tgts_file_path = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        cls.tgts = parsers.read_tgts(tgts_file_path)

        # Read the rest configuration file
        cls.test_config = parsers.read_test_config(
                os.path.join(files_dir, 'test_config.json')
            )

        # Read the tunnel vals
        tunnel_vals = {"ALPHA" : 0.0, "BETA" : 0.0, "PHI" : 0.0, "STRUTZ" : 0.0}
        
        # Get the visibility checker from the grid
        cls.grd = os.path.join(files_dir, 'fml_tc3_volume.grid')
        cls.vis_checker = visibility.VisibilityChecker(
                cls.grd, cls.test_config['oblique_angle']
        )

        # Load the frame
        frame_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )
        cls.frame = cv2.imread(frame_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH)

        # Get the camera calibration for this camera
        cam_tun_path = os.path.join(
            files_dir, 'camera-tunnel-calibration', 'camera01_35_6.json'
        )
        camera_cal_tunnel = parsers.read_camera_tunnel_cal(
            cam_tun_path, cls.frame.shape
        )
        __, __, cls.cameraMatrix, cls.distCoeffs = camera_cal_tunnel
        cls.rmat, cls.tvec = camera_tunnel_calibrate.tf_camera_tgts_thru_tunnel(
            camera_cal_tunnel, tunnel_vals, cls.test_config
        )

        cls.tgts_visible = photogrammetry.get_visible_targets(
            cls.rmat,
            cls.tvec,
            cls.cameraMatrix,
            cls.distCoeffs,
            cls.tgts,
            cls.vis_checker
        )

    def setUp(self):
        """
        Checks files are set up properly before each running test
        """

        file_structure.setUp(files_dir)
    
    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()
    
    def test_get_target_node_idxs(self):
        """ Regression test for get_target_node_idxs

        The solutions were manually verified visually with the debug outputs
        """
        verts, normals = self.vis_checker.get_tvecs_and_norms()
        tgt_vert_idxs = patching.get_target_node_idxs(verts, self.tgts_visible, 0.1)
        
        self.assertTrue(526 * 0.95 < len(tgt_vert_idxs) < 526 * 1.05)

    def test_patchFiducials(self):
        """ Regression test for patchFiducials

        The solutions were manually verified visually with the debug outputs
        """
        internal_intensities_gt = [[106.03251,  106.21941],
                                   [107.01004,  107.28066],
                                   [121.48262,  121.870865],
                                   [123.34717,  123.45227],
                                   [122.19859,  123.282555],
                                   [106.937965, 107.00953],
                                   [130.03972,  125.14807],
                                   [102.69896,  102.12065],
                                   [7.301373,     7.8519864],
                                   [130.2384,   130.08713],
                                   [155.92165,  156.675],
                                   [134.46587,  134.05843],
                                   [125.855385, 125.172966],
                                   [114.238075, 115.599174],
                                   [98.09196,    97.20015], 
                                   [89.30197,    90.37857],
                                   [84.49397,    85.29867],
                                   [74.0,        76.0],
                                   [73.0,        71.0]]

        # Define patch parameters
        boundary_thickness = 2
        buffer_thickness_in = 0.1
        
        out_img = patching.patchFiducials(
            self.tgts_visible,
            self.frame,
            self.rmat,
            self.tvec,
            self.cameraMatrix,
            self.distCoeffs,
            boundary_thickness,
            buffer_thickness_in
        )
        
        # Generate image with all targets highlighted
        ny, nx = self.frame.shape
        xs, ys = np.meshgrid(np.arange(nx), np.arange(ny))
        pts = np.stack((xs, ys), axis=2)
                
        all_targs = np.zeros(self.frame.shape)
        for i, fiducial in enumerate(self.tgts_visible):            
            tgt_proj, targ_size_px, __, __ = patching.get_fiducial_pixel_properties(
                    fiducial,
                    self.rmat,
                    self.tvec,
                    self.cameraMatrix,
                    self.distCoeffs,
                    boundary_thickness,
                    buffer_thickness_in
                )
            
            tgt_proj_formatted = np.array(tgt_proj['proj']).reshape(1, 1, 2)
            dists = np.linalg.norm(pts - tgt_proj['proj'], axis=2)
            
            internals = np.array(np.where(dists <= targ_size_px/2))

            # If there are no internals (ie the target is outside the image) then
            #   the input image should not have been changed
            if internals.shape[1] == 0:
                continue

            np.testing.assert_array_almost_equal(
                    out_img[internals.T[0][0]][internals.T[0][1]],
                    internal_intensities_gt[i][0],
                    decimal=4
                )

            np.testing.assert_array_almost_equal(
                    out_img[internals.T[17][0]][internals.T[17][1]],
                    internal_intensities_gt[i][1],
                    decimal=4
                )

    def test_clusterFiducials(self):
        """ Regression test for clusterFiducials

        The solutions were manually verified visually with the debug outputs
        """
        # Define patch parameters
        boundary_thickness = 2
        buffer_thickness_in = 0.03
        
        # Create a copy of the visible targets to make a cluster
        tgts_visible_cp = copy.deepcopy(self.tgts_visible)
        fake_tgt = copy.deepcopy(tgts_visible_cp[-1])
        fake_tgt['tvec'][0] += 0.1
        tgts_visible_cp.append(fake_tgt)

        # Cluster the fiducials
        fiducial_clusters = patching.clusterFiducials(
                tgts_visible_cp,
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs,
                boundary_thickness,
                buffer_thickness_in
            )

        # Check that none of the clusters overlap
        internals_set = set()

        bounds_set = set()
        for i, cluster in enumerate(fiducial_clusters):
            internals, bounds = patching.get_cluster_internal_and_boundary(
                    cluster,
                    self.rmat,
                    self.tvec,
                    self.cameraMatrix,
                    self.distCoeffs,
                    boundary_thickness,
                    buffer_thickness_in
                )

            # Check if the internals of this cluster overlap with any previous clusters
            # Check if the internals overlap with any boundaries
            for internal in internals:
                internal_tuple = tuple(internal.tolist())

                # Assert that the internal is not in the bounds
                self.assertFalse(internal_tuple in bounds_set)

                # Assert that the internal is not in another internal
                self.assertFalse(internal_tuple in internals_set)

                # Add this internal to the internals set
                internals_set.add(internal_tuple)
            
            # Check if the boundaries of this cluster overlap with any previous clusters
            # Check if the bounds overlap with any internals
            for bound in bounds:
                bound_tuple = tuple(bound.tolist())
                
                # Assert that the bounds are not in the internal
                self.assertFalse(bound_tuple in internals_set)
                
                # Add this bound to the boundary set
                bounds_set.add(bound_tuple)

        # Make sure there are the correct number of clusters
        self.assertEqual(len(fiducial_clusters), 24)
        
        # Make sure the clusters have the correct number of fiducials
        self.assertEqual(len(fiducial_clusters[0]), 2)
        for fiducial_cluster in fiducial_clusters[1:]:
            self.assertEqual(len(fiducial_cluster), 1)
            
    def test_get_cluster_internal_and_boundary(self):
        """ Regression test for get_cluster_internal_and_boundary

        The solutions were manually verified visually with the debug outputs
        """
        boundary_thickness = 2
        buffer_thickness_in = 0.03
        tgts_sub = copy.deepcopy([self.tgts_visible[x] for x in [10, 12, 13, 21]])

        spot_check_internals = [[(( 6, 15), 1), (( 2,  3), 0)],
                                [(( 8,  5), 1), ((12, 20), 1)],
                                [((12, 10), 0), (( 2,  5), 0)],
                                [((10, 10), 1), ((12, 20), 0)]]
        spot_check_bounds    = [[((10,  5), 0), ((10,  1), 1)],
                                [((12,  0), 0), ((14, 15), 0)],
                                [(( 2,  5), 1), (( 6, 15), 0)],
                                [(( 0,  5), 1), ((12, 25), 1)]]
        
        # Check that the function works for single fiducial clusters
        for i, tgt in enumerate(tgts_sub):
            internals_f, bounds_f = patching.get_fiducial_internal_and_boundary(
                tgt,
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs,
                boundary_thickness,
                buffer_thickness_in
            )

            internals_f_set = set()
            for internal_f in internals_f:
                internals_f_set.add(tuple(internal_f.tolist()))

            internals_c, bounds_c = patching.get_cluster_internal_and_boundary(
                    [tgt],
                    self.rmat,
                    self.tvec,
                    self.cameraMatrix,
                    self.distCoeffs,
                    boundary_thickness,
                    buffer_thickness_in
                )

            internals_c_set = set()
            for internal_c in internals_c:
                internals_c_set.add(tuple(internal_c.tolist()))

            np.testing.assert_equal(
                len(internals_f_set),
                len(internals_f_set.intersection(internals_c_set))
            )

        # Check that the function works for multi fiducial clusters
        for i, tgt in enumerate(tgts_sub):
            tgt_copy = copy.deepcopy(tgt)
            tgt_copy['tvec'][0] += 0.20 # moves fiducial in image x (mostly)
            tgt_copy['tvec'][2] += 0.06 # moves fiducial in image y (mostly)

            internals, bounds = patching.get_cluster_internal_and_boundary(
                    [tgt, tgt_copy],
                    self.rmat,
                    self.tvec,
                    self.cameraMatrix,
                    self.distCoeffs,
                    boundary_thickness,
                    buffer_thickness_in
                )

            # print the debugs
            min_bounds = (np.min(bounds[:,0]), np.min(bounds[:,1]))
            max_bounds = (np.max(bounds[:,0]), np.max(bounds[:,1]))
            
            internals_map = np.zeros((max_bounds[1] - min_bounds[1] + 1, 
                                      max_bounds[0] - min_bounds[0] + 1))
          
            for in_pt in internals:
                local_in_pt = (in_pt[0] - min_bounds[0], in_pt[1] - min_bounds[1])
                internals_map[local_in_pt[1]][local_in_pt[0]] = 1
                        
            bounds_map = np.zeros(
                (max_bounds[1] - min_bounds[1] + 1, max_bounds[0] - min_bounds[0] + 1)
            )
            for bd_pt in bounds:
                local_bd_pt = (bd_pt[0] - min_bounds[0], bd_pt[1] - min_bounds[1])
                bounds_map[local_bd_pt[1]][local_bd_pt[0]] = 1
            
            np.testing.assert_array_less(internals_map + bounds_map, 2)
            for spot_check in spot_check_internals[i]:
                np.testing.assert_equal(internals_map[spot_check[0]], spot_check[1])
                
            for spot_check in spot_check_bounds[i]:
                np.testing.assert_equal(bounds_map[spot_check[0]], spot_check[1])

    def test_get_fiducial_internal_and_boundary(self):
        """ Regression test for get_fiducial_internal_and_boundary

        The solutions were manually verified visually with the debug outputs
        """        
        tgts_sub = copy.deepcopy([self.tgts_visible[x] for x in [10, 12, 13]])

        spot_check_internals = [[(( 5, 10), 1), ((20,  2), 0)],
                                [((10, 10), 1), ((20, 20), 0)],
                                [((20, 20), 0), (( 2, 20), 0)]]
        spot_check_bounds    = [[(( 5,  2), 1), (( 5, 10), 0)],
                                [((20,  5), 1), ((20, 20), 1)],
                                [((15,  1), 1), ((20,  5), 1)]]

        for i, tgt in enumerate(tgts_sub):
            internals, bounds = patching.get_fiducial_internal_and_boundary(
                    tgt,
                    self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs,
                    2, 0.1
                )
            
            # print the debugs
            min_bounds = (np.min(bounds[:,0]), np.min(bounds[:,1]))
            max_bounds = (np.max(bounds[:,0]), np.max(bounds[:,1]))          

            internals_map = np.zeros((max_bounds[1] - min_bounds[1] + 1, 
                                      max_bounds[0] - min_bounds[0] + 1))

            for in_pt in internals:
                local_in_pt = (in_pt[0] - min_bounds[0], in_pt[1] - min_bounds[1])
                internals_map[local_in_pt[1]][local_in_pt[0]] = 1

            bounds_map = np.zeros(
                (max_bounds[1] - min_bounds[1] + 1, max_bounds[0] - min_bounds[0] + 1)
            )
            for bd_pt in bounds:
                local_bd_pt = (bd_pt[0] - min_bounds[0], bd_pt[1] - min_bounds[1])
                bounds_map[local_bd_pt[1]][local_bd_pt[0]] = 1
            
            np.testing.assert_array_less(internals_map + bounds_map, 2)
            for spot_check in spot_check_internals[i]:
                np.testing.assert_equal(internals_map[spot_check[0]], spot_check[1])
                
            for spot_check in spot_check_bounds[i]:
                np.testing.assert_equal(bounds_map[spot_check[0]], spot_check[1])

    def test_get_fiducial_boundary_map_from_internal_map(self):
        """ Regression test for get_fiducial_boundary_map_from_internal_map

        The solutions were manually verified visually with the debug outputs
        """         
        boundary_thickness = 2
        def get_fiducial_boundary_map_from_internal_map_regr(internal_map):
            # Dilate the internals image to get an image of the internals and boundary
            kernel = np.full((3, 3), 1)
            int_and_bound_map = copy.deepcopy(internal_map).astype(dtype=np.uint8)
            for i in range(boundary_thickness):
                int_and_bound_map = cv2.dilate(int_and_bound_map, kernel)

            # Get the boundary by subtracting the internals
            boundary_map = int_and_bound_map - internal_map
            return boundary_map
        
        for i, tgt in enumerate(self.tgts_visible):
            internals, bounds = patching.get_fiducial_internal_and_boundary(
                    tgt,
                    self.rmat,
                    self.tvec,
                    self.cameraMatrix,
                    self.distCoeffs,
                    boundary_thickness,
                    0.1
                )
            
            internals_map = np.zeros(self.frame.shape)
            for internal in internals:
                x, y = internal
                if x < 0 or x > self.frame.shape[1]:
                    continue
                if y < 0 or y > self.frame.shape[0]:
                    continue
                internals_map[y][x] = 1
            
            bounds_map = np.zeros(self.frame.shape)
            for bound in bounds:
                x, y = bound
                if x < 0 or x > self.frame.shape[1]:
                    continue
                if y < 0 or y > self.frame.shape[0]:
                    continue
                bounds_map[y][x] = 1

            np.testing.assert_array_equal(
                    bounds_map,
                    get_fiducial_boundary_map_from_internal_map_regr(internals_map)
                )

    def test_get_fiducial_pixel_properties(self):
        """ Regression test for get_fiducial_pixel_properties

        The solutions were manually verified visually with the debug outputs
        """ 
        targ_size_px_gt = [11.442733103728068, 11.441378433873822, 11.738253646918416,
                           11.764717542333987, 11.801661542160153, 11.720571915645861,
                           11.94827679861624,  11.745977350478295, 13.478404742018785,
                           11.843501749982838, 12.05934648991549,  11.96534823545594,
                           11.782260632481044, 11.794715194262851, 11.401202760114563,
                           11.418828875986328, 11.272951268319456, 10.909527128683477,
                           10.911584769089375, 10.329888893588022, 10.372276760939084,
                           9.793731185954318,   9.289072105360367,  9.29690065203498]
        tgt_proj_gt =    [[1062.1596, 262.6362], [1049.5059, 602.7571],
                          [ 958.7569, 401.9505], [ 912.806,  277.4822],
                          [ 896.4405, 540.9792], [ 833.6571, 147.9847],
                          [ 824.7844, 397.0226], [ 806.4481, 686.5303],
                          [ 730.9645, 406.4167], [ 608.0831, 151.1053],
                          [ 608.3256, 398.896 ], [ 609.248,  589.4952],
                          [ 375.6581, 260.3785], [ 366.0181, 537.7377],
                          [ 206.7432, 213.7723], [ 203.3067, 594.6966],
                          [ 113.9483, 397.4618], [ -0.847,   328.4902],
                          [   0.6552, 512.2042], [-158.6912, 252.179 ],
                          [-154.1934, 544.9812], [-318.6646, 390.3488],
                          [-430.5195, 231.9778], [-436.5539, 557.0394]]

        # Define patch parameters
        boundary_thickness = 2
        buffer_thickness_in = 0.05

        for i, fiducial in enumerate(self.tgts_visible):
            tgt_proj, targ_size_px, __, __ = patching.get_fiducial_pixel_properties(
                    fiducial, self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs,
                    boundary_thickness, buffer_thickness_in
                )
            np.testing.assert_almost_equal(targ_size_px_gt[i], targ_size_px, decimal=2)
            np.testing.assert_almost_equal(tgt_proj_gt[i], tgt_proj['proj'], decimal=2)

    def test_polyfit2D(self):
        """ Regression test for polyfit2D

        The solutions were manually verified visually with the debug outputs
        """        
        coeff_gts = [[ 1.6494e+02, -1.8998e+00,  1.4664e-01, -4.0182e-03, -1.3818e+00,
                       1.0423e-01, -6.6948e-04,  1.0484e-01, -3.5790e-03, -2.2306e-03],
                     [ 1.3130e+02, -1.4366e+00,  1.3644e-01, -3.3798e-03, -9.3938e-01,
                      -1.1125e-02, -3.4015e-04,  1.2540e-01,  1.9413e-03, -4.2443e-03],
                     [ 1.1783e+02, -8.8762e-01,  6.5684e-02, -8.7675e-04, -1.0222e-01,
                       4.5181e-03, -1.7885e-03,  2.3825e-02,  1.5239e-03, -1.2054e-03]]
        
        tgts_sub = copy.deepcopy([self.tgts_visible[x] for x in [10, 12, 13]])

        for i, tgt in enumerate(tgts_sub):            
            # Get the bounds
            internals, bounds = patching.get_fiducial_internal_and_boundary(
                tgt, self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs, 2, 0.1)
            
            # Get the intensities of the bounds
            Is = [self.frame[bound[1]][bound[0]] for bound in bounds]

            # Convert the bounds to local bounds
            min_bounds = (np.min(bounds[:,0]), np.min(bounds[:,1]))
            local_bounds = bounds - min_bounds
            local_internals = internals - min_bounds

            # fit a 2D polynomial to the bounds
            coeffs = patching.polyfit2D(local_bounds, Is)
            np.testing.assert_almost_equal(coeff_gts[i], coeffs, decimal=2)
    
    def test_polyval2D(self):
        """ Regression test for polyval2D

        The solutions were manually verified visually with the debug outputs
        """ 
        # Define patch parameters
        boundary_thickness = 2
        buffer_thickness_in = 0.05

        internal_intensities_gts = [[157.0578855264, 156.61471620885, 158.33528562904],
                                    [125.6815396239, 125.39472392504, 126.34666003293],
                                    [115.0199014473, 115.84445115081, 115.74647936438]]
        tgts_sub = copy.deepcopy([self.tgts_visible[x] for x in [10, 12, 13]])

        for i, tgt in enumerate(tgts_sub):
            # Get the bounds
            internals, bounds = patching.get_fiducial_internal_and_boundary(
                tgt, self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs, 2, 0.05)
            
            # Get the intensities of the bounds
            Is = [self.frame[bound[1]][bound[0]] for bound in bounds]

            # Convert the bounds to local bounds
            min_bounds = (np.min(bounds[:,0]), np.min(bounds[:,1]))
            local_bounds = bounds - min_bounds
            local_internals = internals - min_bounds

            # fit a 2D polynomial to the bounds
            coeffs = patching.polyfit2D(local_bounds, Is)

            # Get the internal intensities based on the fit
            internal_intensities = patching.polyval2D(local_internals, coeffs)
            np.testing.assert_almost_equal(
                internal_intensities_gts[i][0], internal_intensities[ 0], decimal=4
            )
            np.testing.assert_almost_equal(
                internal_intensities_gts[i][1], internal_intensities[ 7], decimal=4
            )
            np.testing.assert_almost_equal(
                internal_intensities_gts[i][2], internal_intensities[23], decimal=4
            )


if __name__ == "__main__":
    unittest.main()
