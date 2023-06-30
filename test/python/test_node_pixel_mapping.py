import unittest
import numpy as np
import cv2
import os

import file_structure
from upsp.cam_cal_utils import (
    camera_tunnel_calibrate,
    parsers,
    visibility,
)
from upsp.intensity_mapping import node_pixel_mapping

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class NodePixelMappingTestCase(unittest.TestCase):
    """
    Tests for node-pixel mapping
    """
    
    @classmethod
    def setUpClass(cls):
        # Read tgts file
        tgts_file_path = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        cls.tgts_all = parsers.read_tgts(tgts_file_path, output_target_types=None)
        cls.tgts_dot = parsers.read_tgts(tgts_file_path, output_target_types='dot')

        # Read the rest configuration file
        cls.test_config = {"oblique_angle" : 70,
                           "tunnel-cor_to_tgts_tvec" : np.zeros((3, 1)),
                           "tunnel-cor_to_tgts_rmat": np.eye(3),
                           "tunnel-cor_to_tunnel-origin_tvec": np.zeros((3, 1)),
                           "tunnel-cor_to_tunnel-origin_rmat": np.eye(3),
                           }

        # Read the tunnel vals
        tunnel_vals = {"ALPHA" : 0.0, "BETA" : 0.0, "PHI" : 0.0, "STRUTZ" : 0.0}
        
        # Get the visibility checker from the grid
        grd = os.path.join(files_dir, 'fml_tc3_volume.grid')
        cls.vis_checker = visibility.VisibilityChecker(
            grd, cls.test_config['oblique_angle']
        )
        
        # Load the frame
        frame_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )
        cls.frame = cv2.imread(
            frame_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH
        )

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

        # Get vertex and normal information
        cls.verts, cls.normals = cls.vis_checker.get_tvecs_and_norms()

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

    def test_node_to_pixel_mapping_keyframe(self):
        """ Regression test for node_to_pixel_mapping_keyframe

        The solutions were manually verified visually with the debug outputs
        """
        projs, jacs, vis_idxs = node_pixel_mapping.node_to_pixel_mapping_keyframe(
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs,
                self.vis_checker,
                self.verts,
                self.normals
            )

        self.assertEqual(len(self.verts), projs.shape[0])
        self.assertEqual(len(self.verts), jacs.shape[0])

        not_nans = np.argwhere(np.isnan(projs[:, 0]) == False)
        self.assertEqual(len(not_nans), vis_idxs.shape[0])

        self.assertTrue(148823 * 0.95 < len(not_nans) < 148823 * 1.05)
        
    def test_node_to_pixel_mapping_non_keyframe(self):
        """ Regression test for node_to_pixel_mapping_non_keyframe

        The solutions were manually verified visually with the debug outputs
        """
        projs_key, jacs, vis_idxs = node_pixel_mapping.node_to_pixel_mapping_keyframe(
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs,
                self.vis_checker,
                self.verts,
                self.normals
            )

        proj_grad = node_pixel_mapping.node_to_pixel_mapping_non_keyframe(
                self.rmat,
                self.tvec,
                self.rmat,
                self.tvec,
                projs_key,
                jacs,
                vis_idxs
            )

        np.testing.assert_allclose(projs_key, proj_grad, atol=0.5)

    def test_node_to_pixel_mapping_non_keyframe_full(self):
        """ Regression test for node_to_pixel_mapping_non_keyframe_full

        The solutions were manually verified visually with the debug outputs
        """
        projs_key, jacs, vis_idxs = node_pixel_mapping.node_to_pixel_mapping_keyframe(
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs,
                self.vis_checker,
                self.verts,
                self.normals
            )

        projs_full = node_pixel_mapping.node_to_pixel_mapping_non_keyframe_full(
                self.rmat,
                self.tvec,
                self.cameraMatrix,
                self.distCoeffs, 
                self.verts,
                vis_idxs
            )

        np.testing.assert_equal(projs_full, projs_key)


if __name__ == "__main__":
    unittest.main()
