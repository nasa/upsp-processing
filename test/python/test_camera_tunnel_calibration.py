import unittest
import numpy as np
import os
import sys
import cv2

import file_structure
from upsp.cam_cal_utils import (
    camera_tunnel_calibrate as ctc,
    external_calibrate,
    parsers,
    photogrammetry,
    visibility,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')
ctc_dir = os.path.join(files_dir, 'output')

# If you are debug, setting this to True will unmute the function calls.
# If this is False (by default), then function calls will be muted
test_debug = False


class CameraTunnelCalibrationTestCase(unittest.TestCase):
    """
    Tests for the camera_tunnel_calibrate module
    """

    @classmethod
    def setUpClass(cls):
        pass
        
    def setUp(self):
        """
        Checks files set up properly before each running test
        """
        # Check file structure
        file_structure.setUp(files_dir)

        ctc.debug_show_img_targets = False
        ctc.debug_show_matches = False
        ctc.debug_show_3D_targets = False

        external_calibrate.debug_raw_matches = False
        external_calibrate.debug_coarse_optimization = False
        external_calibrate.debug_visible_projections = False
        external_calibrate.debug_refined_optimization = False
        external_calibrate.debug_show_localizations = False
     
    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()
    
    def test_camera_to_tunnel_calibrate(self):
        """
        Regression test for camera_to_tunnel_calibrate
        """
        if not test_debug:
            f = open(os.devnull, 'w')
            std_stdout_orig = sys.stdout
            sys.stdout = f
        
        # Get pathings to the data
        tgts_filepath = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        grid_filepath = os.path.join(files_dir, 'fml_tc3_volume.grid')
        test_config_filepath = os.path.join(files_dir, 'test_config.json')
        
        # Read tgts file and wtd file
        tgts_all = parsers.read_tgts(tgts_filepath)

        # Read in the test_config file
        test_config = parsers.read_test_config(test_config_filepath)
        
        # Get the visibility checker from the grid
        vis_checker = visibility.VisibilityChecker(grid_filepath,
                                                   test_config['oblique_angle'])
        
        # Populate the model's tunnel position with all 0's since it was in a lab
        tunnel_vals = {'ALPHA' : 0.0, 'BETA' : 0.0, 'PHI' : 0.0, 'STRUTZ' : 0.0}

        # Run the calibration script
        img_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH)

        # Load the internal calibration for this camera
        incal_path = os.path.join(
            files_dir, 'internal-calibration', 'camera01_35_6.json'
        )
        internal_cal = parsers.read_internal_params(
            incal_path, img.shape, read_sensor=True
        )
        
        # Read in manual bboxes
        manual_det_filename = 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.xml'
        manual_detections_path = os.path.join(
            files_dir, 'detection', 'Annotations', manual_det_filename
        )
        manual_detection = parsers.read_pascal_voc(manual_detections_path)

        ctc.camera_to_tunnel_calibrate(
            ctc_dir,
            [img],
            [internal_cal],
            [manual_detection],
            tunnel_vals,
            tgts_all,
            test_config,
            vis_checker
        )

        # Compare the generated camera calibrations to the baseline. The baseline 
        #   solutions were manually verified visually with the debug outputs
        regression_path = os.path.join(
            files_dir, 'camera-tunnel-calibration', 'camera01_35_6.json'
        )
        regression_cal = parsers.read_camera_tunnel_cal(regression_path, (512, 1024))
        rmat_reg, tvec_reg, cameraMatrix_reg, distCoeffs_reg = regression_cal

        output_path = os.path.join(ctc_dir, 'camera01.json')
        output_cal = parsers.read_camera_tunnel_cal(output_path, (512, 1024))
        rmat_ctc, tvec_ctc, cameraMatrix_ctc, distCoeffs_ctc = output_cal

        np.testing.assert_array_almost_equal(rmat_reg, rmat_ctc, decimal=1)
        np.testing.assert_array_almost_equal(tvec_reg, tvec_ctc, decimal=1)
        np.testing.assert_array_almost_equal(cameraMatrix_reg, cameraMatrix_ctc, decimal=1)
        np.testing.assert_array_almost_equal(distCoeffs_reg, distCoeffs_ctc, decimal=1)

        if not test_debug:
            sys.stdout = std_stdout_orig
            f.close()

        # Deleted the generated camera_tunnel_calibration json file
        ctc_json = os.path.join(ctc_dir, 'camera01.json')
        os.remove(ctc_json)

    def test_tunnel_transform(self):
        """
        Regression test for tunnel_transform
        
        The solutions were manually verified visually with the debug outputs
        """
        
        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)
        
        # This is a cached version of the function. Should the function be changed in
        #   the future, we want to ensure consistency between the old and new function
        def regression_tunnel_transform(ALPHA, BETA, PHI, STRUTZ, sting_offset):
            alpha_rad = np.deg2rad(ALPHA)
            beta_rad = np.deg2rad(BETA)
            phi_rad = np.deg2rad(PHI)

            c_alpha = np.cos(alpha_rad)
            s_alpha = np.sin(alpha_rad)
            pitch = np.array([[c_alpha, 0, -s_alpha], [0, 1, 0], [s_alpha, 0, c_alpha]])

            c_beta = np.cos(beta_rad)
            s_beta = np.sin(beta_rad)
            yaw = np.array([[c_beta, s_beta, 0], [-s_beta, c_beta, 0], [0, 0, 1]])

            c_phi = np.cos(phi_rad)
            s_phi = np.sin(phi_rad)
            roll = np.array([[1, 0, 0], [0, c_phi, -s_phi], [0, s_phi, c_phi]])

            rotation_matrix = np.matmul(pitch, np.matmul(yaw, roll))
            rotation_matrix = np.linalg.inv(rotation_matrix)

            tvec__cor_tgts__tgts_frame = sting_offset

            tvec__knuckle_tgts = np.matmul(rotation_matrix, tvec__cor_tgts__tgts_frame)
            tvec__tunnel_tgts__tunnel_frame = tvec__knuckle_tgts + np.array([0, 0, STRUTZ])

            return rotation_matrix, tvec__tunnel_tgts__tunnel_frame.reshape(3, 1)
    
        # Generate 50 random inputs
        alphas =  (360 * rng.random(50)) - 180
        betas =   (360 * rng.random(50)) - 180
        phis =    (360 * rng.random(50)) - 180
        strutzs = (200 * rng.random(50)) - 100
        s_os =    (200 * rng.random((50, 3))) - 100

        # Test that the output of photogrammtery.tunnel_transformation matches the
        #   cached version for all the random inputs
        for alpha, beta, phi, strutz, s_o in zip(alphas, betas, phis, strutzs, s_os):
            p_tt = ctc.tunnel_transform(alpha, beta, phi, strutz, s_o.reshape(3, 1))            
            r_tt = regression_tunnel_transform(alpha, beta, phi, strutz, s_o)

            np.testing.assert_array_almost_equal(p_tt[0], r_tt[0], decimal=3)
            np.testing.assert_array_almost_equal(p_tt[1], r_tt[1], decimal=3)
            
            np.testing.assert_array_almost_equal(p_tt[0].shape, (3, 3))
            np.testing.assert_array_almost_equal(p_tt[1].shape, (3, 1))

    def test_tf_camera_tgts_thru_tunnel(self):
        """
        Regression test of tf_camera_tgts_thru_tunnel
        
        The solutions were manually verified visually with the debug outputs
        """
        
        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)
        
        # This is a cached version of the function. Should the function be changed in
        #   the future, we want to ensure consistency between the old and new function
        def regression_tf_camera_tgts_thru_tunnel(camera_cal, wtd, test_config): 
            # Turn the wind tunnel data into the transformation from tunnel to targets
            wtd_transform = ctc.tunnel_transform(wtd['ALPHA'], wtd['BETA'], wtd['PHI'], 
                                                 wtd['STRUTZ'], test_config['tunnel-cor_to_tgts_tvec'])
            rmat_tunnel_tgts, tvec_tunnel_tgts = wtd_transform

            # Transformation from tgts frame to runnel frame
            rmat_tgts_tunnel = np.linalg.inv(rmat_tunnel_tgts)
            tvec_tgts_tunnel = -np.matmul(rmat_tgts_tunnel, tvec_tunnel_tgts)

            # Decompose the camera calibration into its parts
            rmat__camera_tunnel, tvec__camera_tunnel, cameraMatrix, distCoeffs = camera_cal

            # Combine the transformations to get the transformation from `camera to tgts frame
            rmat__camera_tgts = np.matmul(rmat__camera_tunnel, np.linalg.inv(rmat_tgts_tunnel))
            tvec__camera_tgts = tvec__camera_tunnel + np.matmul(rmat__camera_tunnel,
                                                                tvec_tunnel_tgts)

            return rmat__camera_tgts, tvec__camera_tgts.reshape(3, 1)
        
        # Generate 50 random inputs
        strutzs = (200 * rng.random(50)) - 100
        s_os =    (200 * rng.random((50, 3))) - 100

        # Test 50 random inputs
        for i in range(50):
            cameraMatrix = np.eye(3)
            cameraMatrix[0, 2] = 512
            cameraMatrix[1, 2] = 256
            distCoeffs = (2 * rng.random(5)) - 1
            
            # Generate a random but valid rotation matrix
            angles = (360 * rng.random(3)) - 180
            rotx = photogrammetry.rot(angles[0], 'x')
            roty = photogrammetry.rot(angles[1], 'y')
            rotz = photogrammetry.rot(angles[2], 'z')
            rmat = np.matmul(rotz, np.matmul(roty, rotx))

            # Generate a random translation vector
            tvec = (200 * rng.random((3, 1))) - 100

            # Package the camera calibration
            camera_cal = rmat, tvec, cameraMatrix, distCoeffs 

            tunnel_angles = (360 * rng.random(3)) - 180
            wtd = {'ALPHA' : tunnel_angles[0],
                   'BETA' : tunnel_angles[1],
                   'PHI' : tunnel_angles[2],
                   'STRUTZ' : strutzs[i]}
            test_config = {'tunnel-cor_to_tgts_tvec' : s_os[i].reshape(3, 1)}
            
            # Test the tf_camera_tgts_thru_tunnel against the cached function
            reg = regression_tf_camera_tgts_thru_tunnel(camera_cal, wtd, test_config)
            pho = ctc.tf_camera_tgts_thru_tunnel(camera_cal, wtd, test_config)

            np.testing.assert_array_almost_equal(reg[0], pho[0], decimal=3)
            np.testing.assert_array_almost_equal(reg[1], pho[1], decimal=3)
            
            np.testing.assert_array_almost_equal(pho[0].shape, (3, 3))
            np.testing.assert_array_almost_equal(pho[1].shape, (3, 1))

if __name__ == "__main__":
    unittest.main()
