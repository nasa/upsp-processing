import unittest
import numpy as np
import cv2
import os
import sys
import copy

import file_structure
from upsp.target_operations import target_detection
from upsp.cam_cal_utils import (
    external_calibrate,
    img_utils,
    parsers,
    photogrammetry,
    visibility,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')

# If you are debug, setting this to True will unmute the function calls.
# If this is False (by default), then function calls will be muted
test_external_calibrate_debug = False

class ExternalCalibrateTestCase(unittest.TestCase):
    """
    Tests for external calibrate
    """
    
    @classmethod
    def setUpClass(cls):      
        # Read in the image
        img_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )
        cls.img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH)

        # Get the camera calibration for this camera
        cam_tun_path = os.path.join(
            files_dir, 'camera-tunnel-calibration', 'camera01_35_6.json'
        )
        camera_cal_tunnel = parsers.read_camera_tunnel_cal(
            cam_tun_path, cls.img.shape
        )
        cls.rmat, cls.tvec, cls.cameraMatrix, cls.distCoeffs = camera_cal_tunnel

        # Read the rest configuration file
        test_config_file = os.path.join(files_dir, 'test_config.json')
        cls.test_config = parsers.read_test_config(test_config_file)
        cls.test_config["crosscorr_coeff"] = 0.5
        
        # Get the visibility checker from the grid
        grd_file = os.path.join(files_dir, 'fml_tc3_volume.grid')
        cls.vis_checker = visibility.VisibilityChecker(
            grd_file, 70, epsilon=1e-4
        )

        # Read in the targets file
        tgts_file = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        cls.tgts = parsers.read_tgts(tgts_file)

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
        test_external_calibrate_debug = False

        # Set debugs to False
        external_calibrate.debug_raw_matches = False
        external_calibrate.debug_coarse_optimization = False
        external_calibrate.debug_refined_matches = False
        external_calibrate.debug_visible_projections = False
        external_calibrate.debug_refined_optimization = False

        file_structure.setUp(files_dir)

    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()

    def test_compare_poses(self):
        """
        Regression test for compare_poses

        Many randomly generated inputs tests known properties of the outputs
        """

        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)
        
        # Generate 100 random inputs
        rotation = (360 * rng.random(600)) - 180
        
        xs = (100 * rng.random(200)) - 50
        ys = (100 * rng.random(200)) - 50
        zs = (100 * rng.random(200)) - 50
        
        for i in range(100):
            # Get random rotation matries
            I = np.eye(3)
            R_a1 = np.matmul(photogrammetry.rot(rotation[6 * i + 0], 'x'), I)
            R_a2 = np.matmul(photogrammetry.rot(rotation[6 * i + 1], 'y'), R_a1)
            R_a3 = np.matmul(photogrammetry.rot(rotation[6 * i + 2], 'z'), R_a2)
            
            R_b1 = np.matmul(photogrammetry.rot(rotation[6 * i + 3], 'x'), I)
            R_b2 = np.matmul(photogrammetry.rot(rotation[6 * i + 4], 'y'), R_b1)
            R_b3 = np.matmul(photogrammetry.rot(rotation[6 * i + 5], 'z'), R_b2)

            # Get random translation vectors
            x1, y1, z1 = xs[2 * i], ys[2 * i], zs[2* i]
            x2, y2, z2 = xs[2 * i + 1], ys[2 * i + 1], zs[2 * i + 1]

            # Calculate correct theta and dist
            rmat_rel = np.matmul(R_b3, R_a3.T)
            tvec_tf = -np.matmul(rmat_rel, np.array([x2, y2, z2]))
            tvec_rel = np.array([x1, y1, z1]) + tvec_tf
            dist_gt = np.linalg.norm(tvec_rel)
            rvec_rel, _ = cv2.Rodrigues(rmat_rel)
            theta_gt_rad = np.linalg.norm(rvec_rel)
            theta_gt = np.rad2deg(theta_gt_rad)
            
            # Calculate theta and dist with excal
            theta, dist = external_calibrate.compare_poses(
                [R_a3, np.array([x2, y2, z2])], [R_b3, np.array([x1, y1, z1])]
            )

            # Compare to ensure consistency
            np.testing.assert_array_almost_equal(theta, theta_gt, decimal=4)
            np.testing.assert_array_almost_equal(dist,  dist_gt,  decimal=4)

    def test_external_calibrate(self):
        """
        Regression test of external_calibrate
        
        Outputs were manually verified to be as expected
        """ 
        # Initialize input values        
        rmat = np.matmul(photogrammetry.rot(0.25, 'x'), self.rmat)
        rmat = np.matmul(photogrammetry.rot(-0.25, 'y'), rmat)
        tvec = self.tvec + np.array([[0.02], [-0.01], [0.02]])
        camera_tunnel_cal = [rmat, tvec, self.cameraMatrix, self.distCoeffs]
        
        # Scale the image
        img8bit = img_utils.scale_image_max_inlier(self.img)

        # Perform template detection to find and match the targets to image locations
        template_detection_output = target_detection.template_detection(
            img8bit,
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            self.tgts,
            self.test_config
        )
        tgts_match, img_targets_match, num_matches = template_detection_output

        # Run the external calibration function
        ex_cal_out = external_calibrate.external_calibrate(
            self.img,
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_match[:num_matches],
            img_targets_match[:num_matches],
            self.vis_checker,
            self.test_config,
            isMatched=False,
            max_localize_delta=None,
            reprojectionError=6.0
        )
        rmat, tvec, tgt_inliers, img_target_inliers = ex_cal_out
        
        rmat_gt = [[-0.9996968381, -0.0131838521,  0.0207946606],
                   [-0.0133185088,  0.999891139,  -0.0063503932],
                   [-0.0207086742, -0.0066254219, -0.9997635994]]
        tvec_gt = [[-5.0804278566], [-0.0481402216], [11.556791127]]

        # Test that the function output closely matches the ground truth
        np.testing.assert_array_almost_equal(rmat, rmat_gt, decimal=3)
        np.testing.assert_array_almost_equal(tvec, tvec_gt, decimal=3)

        # Check that external calibrate raises an error when too few targets are matched
        with self.assertRaises(ValueError):
            ex_cal_out = external_calibrate.external_calibrate(
                self.img,
                self.rmat,
                self.tvec,
                self.cameraMatrix, 
                self.distCoeffs,
                tgts_match[:num_matches][:2],
                img_targets_match[:num_matches][:2],
                self.vis_checker,
                self.test_config,
                isMatched=False,
                max_localize_delta=None,
                reprojectionError=6.0
            )

    def test_check_external_calibrate_two_stage_inputs(self):
        """
        Regression test of check_external_calibrate_two_stage_inputs
        
        Outputs were manually verified to be as expected
        """
        # Direct the std out to nothing temporarily to avoid error messages
        if not test_external_calibrate_debug:
            f = open(os.devnull, 'w')
            std_stdout_orig = sys.stdout
            sys.stdout = f
        
        check_excal = external_calibrate.check_external_calibrate_two_stage_inputs

        #-------------------------------------------------------------------------------
        # Load all proper inputs
        camera_tunnel_cal = [self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs]
        rmat, tvec = self.rmat, self.tvec
        cameraMatrix, distCoeffs = self.cameraMatrix, self.distCoeffs
        incal = camera_tunnel_cal[2:]

        tgts = self.tgts
        vis_checker = self.vis_checker
        img = self.img
        test_config = self.test_config

        # Pass the proper inputs and check for success
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, True)

        # Verify img checks

        # Pass None object
        check_bool = check_excal(
            None, rmat, tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Pass an object with an np.int32 object type
        check_bool = check_excal(
            img.astype(np.int32),
            rmat,
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
                
        # Pass 3 dimensional image. Since it is single channel, it should return True
        img_3 = np.expand_dims(img, axis=2)
        check_bool = check_excal(
            img_3, rmat, tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, True)

        # Pass 3 dimensional image. Since it is 3 channel, it should return False
        check_bool = check_excal(
            np.concatenate((img_3, img_3, img_3), axis=2),
            rmat,
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Pass 4 dimensional image. Should return False
        check_bool = check_excal(
            np.expand_dims(img_3, axis=3),
            rmat,
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        #-------------------------------------------------------------------------------
        # Verify rmat checks

        # Passing a list version for reference as well
        check_bool = check_excal(
            img, rmat.tolist(), tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an rmat with improper dimensions
        rmat_3 = np.expand_dims(rmat, 1)
        check_bool = check_excal(
            img, rmat_3, tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an rmat with improper dimensions
        check_bool = check_excal(
            img,
            [[0.0, 0.0], [0.0, 0.0]],
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an rmat with the wrong datatype (integer)
        check_bool = check_excal(
            img,
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        # Pass an invalid rotation matrix
        check_bool = check_excal(
            img,
            [[1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        #-------------------------------------------------------------------------------
        # Verify tvec checks

        # Passing a list version for reference as well
        check_bool = check_excal(
            img,
            rmat,
            tvec.tolist(),
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing (3,1) version for reference as well
        tvec = tvec.reshape((3, 1))
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, True)
        
        # Passing an tvec with improper dimensions
        check_bool = check_excal(
            img,
            rmat,
            np.expand_dims(tvec, 1),
            incal,
            tgts,
            test_config, 
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an tvec with improper dimensions
        check_bool = check_excal(
            img,
            rmat,
            [[0.0, 0.0], [0.0, 0.0]],
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an tvec with the wrong datatype (integer)
        check_bool = check_excal(
            img,
            rmat,
            [1, 2, 3],
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        #-------------------------------------------------------------------------------
        # Verify incal checks

        # Passing a list version of incal[0] for reference as well
        incal_test = [incal[0].tolist()] + list(incal[1:])
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, True)
        
        # Passing an incal[0] with improper dimensions
        incal_test = [np.expand_dims(incal[0], 1)] + list(incal[1:])
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an incal[0] with improper dimensions
        bad_cm = [np.array([[663.4, 0.0, 512.0], [0.0, 664.3, 1024.0]])]
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an incal[0] with the wrong datatype (integer)
        bad_cm = [np.array([[600, 0, 512], [0, 800, 1024], [0, 0, 1]])]
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        # Pass an invalid incal[0] versions
        bad_cm = np.array([[660.0, 1.0, 512.0], [0.0, 660., 1024.0], [0.0, 0.0, 1.0]])
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        bad_cm = np.array([[660.0, 0.0, 512.0], [1.0, 660., 1024.0], [0.0, 0.0, 1.0]])
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        bad_cm = np.array([[660.0, 0.0, 512.0], [0.0, 660., 1024.0], [1.0, 0.0, 1.0]])
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        bad_cm = np.array([[660.0, 0.0, 512.0], [0.0, 660., 1024.0], [0.0, 1.0, 1.0]])
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        bad_cm = np.array([[660.0, 0.0, 512.0], [0.0, 660., 1024.0], [0.0, 0.0, 2.0]])
        incal_test = [bad_cm, list(incal[1:])]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        # Passing a list version of incal[1] for reference as well
        incal_test = [incal[0]] + [incal[1].tolist()]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, True)
        
        # Passing an incal[1] with improper dimensions
        incal_test = [incal[0]] + [np.expand_dims(incal[1], 1)]
        check_bool = check_excal(
            img,
            rmat,
            tvec,
            incal_test,
            tgts,
            test_config,
            vis_checker,
            None
        )
        self.assertEqual(check_bool, False)

        # Passing an incal[1] with improper dimensions
        incal_test = [incal[0]] + [np.array([663.4, 0.0, 512.0]).reshape((3, 1))]
        check_bool = check_excal(
            img, rmat, tvec, incal_test, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)
        
        # Passing an incal[1] with the wrong datatype (integer)
        incal_test = [incal[0]] + [np.array([[1, 0, 0, 0, 0]])]
        check_bool = check_excal(
            img, rmat, tvec, incal_test, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        #-------------------------------------------------------------------------------
        # Check tgts input

        # Pass a blank list for tgts
        check_bool = check_excal(
            img, rmat, tvec, incal, [], test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Remove tvec from one of the tgts
        tgts_test = copy.deepcopy(tgts)
        del tgts_test[0]['tvec']
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts_test, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Pass a tgts with list instead of numpy arrays
        tgts_test = copy.deepcopy(tgts)
        tgts_test[0]['tvec'] = tgts_test[0]['tvec'].tolist()
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts_test, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, False)
        
        #-------------------------------------------------------------------------------
        # Check test_config input
        
        # Check a float and integer for test_config_test['max_dist']
        #   (both are proper inputs)
        test_config_test = copy.deepcopy(test_config)
        test_config_test['max_dist'] = 32.0
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, True)
        
        test_config_test['max_dist'] = 32
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, True)

        # Passing a string for test_config_test['max_dist'] is improper
        test_config_test['max_dist'] = '32'
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Pasing test_config without a max_dist item is improper
        del test_config_test['max_dist']
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Check a float and integer for test_config_test['min_dist']
        #   (both are proper inputs)
        test_config_test = copy.deepcopy(test_config)
        test_config_test['min_dist'] = 32.0
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, True)
        
        test_config_test['min_dist'] = 32
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, True)

        # Passing a string for test_config_test['min_dist'] is improper
        test_config_test['min_dist'] = '32'
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, False)

        # Pasing test_config without a min_dist item is improper
        del test_config_test['min_dist']
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config_test, vis_checker, None
        )
        self.assertEqual(check_bool, False)
        
        #-------------------------------------------------------------------------------
        # Check vis_checker input
    
        # Pass the wrong vis_checker object
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, 432, None
        )
        self.assertEqual(check_bool, False)

        # Pass the wrong vis_checker object
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, '432', None
        )
        self.assertEqual(check_bool, False)
        
        # Pass the wrong vis_checker object
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, None, None
        )
        self.assertEqual(check_bool, False)

        # Pass the vis_checker with the wrong vis_checker_test.scene object
        vis_checker_test = copy.copy(vis_checker)
        vis_checker_test.scene = None
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker_test, None
        )
        self.assertEqual(check_bool, False)
        
        # Pass the vis_checker with the wrong vis_checker_test.scene object
        vis_checker_test.scene = "BVH"
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker_test, None
        )
        self.assertEqual(check_bool, False)

        #-------------------------------------------------------------------------------
        # Check debug input

        # debug can be None, int, or string. Check those values pass the check
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, None
        )
        self.assertEqual(check_bool, True)
        
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, 2
        )
        self.assertEqual(check_bool, True)
        
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, '3'
        )
        self.assertEqual(check_bool, True)
        
        # Any other object should fail
        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, 43.0
        )
        self.assertEqual(check_bool, False)

        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, [43])
        self.assertEqual(check_bool, False)

        check_bool = check_excal(
            img, rmat, tvec, incal, tgts, test_config, vis_checker, (4, 3)
        )
        self.assertEqual(check_bool, False)

        # Redirect the std out to the original location
        if not test_external_calibrate_debug:
            sys.stdout = std_stdout_orig
            f.close()

    def test_external_calibrate_two_stage_from_wtd(self):
        """
        Regression test of external_calibrate_two_stage_from_wtd
        
        Outputs were manually verified to be as expected
        """
        # Initialize input values
        tunnel_vals = {"ALPHA" : 0.0, "BETA" : 0.0, "PHI" : 0.0, "STRUTZ" : 0.0}
        
        rmat = np.matmul(photogrammetry.rot(0.25, 'x'), self.rmat)
        rmat = np.matmul(photogrammetry.rot(-0.25, 'y'), rmat)
        tvec = self.tvec + np.array([[0.02], [-0.01], [0.02]])
        camera_tunnel_cal = [rmat, tvec, self.cameraMatrix, self.distCoeffs]

        # Direct the std out to nothing temporarily to avoid error messages
        if not test_external_calibrate_debug:
            f = open(os.devnull, 'w')
            std_stdout_orig = sys.stdout
            sys.stdout = f

        # Get the fine-tuned external_calibration output
        rmat_opt, tvec_opt = external_calibrate.external_calibrate_two_stage_from_wtd(
            self.img,
            tunnel_vals,
            camera_tunnel_cal,
            self.tgts,
            self.test_config,
            self.vis_checker
        )

        # Redirect the std out to the original location
        if not test_external_calibrate_debug:
            sys.stdout = std_stdout_orig
            f.close()
        
        # Check that the function output closely matches the expected output
        np.testing.assert_array_almost_equal(rmat_opt, self.rmat, decimal=3)
        np.testing.assert_array_almost_equal(tvec_opt, self.tvec, decimal=3)

    def test_external_calibrate_two_stage(self):
        """
        Regression test of external_calibrate_two_stage
        
        Outputs were manually verified to be as expected
        """
        # Initialize input values        
        rmat = np.matmul(photogrammetry.rot(0.25, 'x'), self.rmat)
        rmat = np.matmul(photogrammetry.rot(-0.25, 'y'), rmat)
        tvec = self.tvec + np.array([[0.02], [-0.01], [0.02]])
        camera_tunnel_cal = [rmat, tvec, self.cameraMatrix, self.distCoeffs]

        # Direct the std out to nothing temporarily to avoid error messages
        if not test_external_calibrate_debug:
            f = open(os.devnull, 'w')
            std_stdout_orig = sys.stdout
            sys.stdout = f

        # Get the fine-tuned external_calibration output
        incal = camera_tunnel_cal[2:]
        rmat_opt, tvec_opt = external_calibrate.external_calibrate_two_stage(
            self.img, rmat, tvec, incal, self.tgts, self.test_config, self.vis_checker
        )

        # Redirect the std out to the original location
        if not test_external_calibrate_debug:
            sys.stdout = std_stdout_orig
            f.close()
        
        # Check that the function output closely matches the expected output
        np.testing.assert_array_almost_equal(rmat_opt, self.rmat, decimal=3)
        np.testing.assert_array_almost_equal(tvec_opt, self.tvec, decimal=3)

    def test_external_calibrate_one_step(self):
        """
        Regression test of external_calibrate_one_step
        
        Outputs were manually verified to be as expected
        """
        # Initialize input values        
        rmat = np.matmul(photogrammetry.rot(0.25, 'x'), self.rmat)
        rmat = np.matmul(photogrammetry.rot(-0.25, 'y'), rmat)
        tvec = self.tvec + np.array([[0.02], [-0.01], [0.02]])
        camera_tunnel_cal = [rmat, tvec, self.cameraMatrix, self.distCoeffs]

        # Scale the image
        img8bit = img_utils.scale_image_max_inlier(self.img)

        # Perform template detection to find and match the targets to image locations
        template_detection_output = target_detection.template_detection(
            img8bit,
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            self.tgts,
            self.test_config
        )
        tgts_match, img_targets_match, num_matches = template_detection_output

        # Do the coarse external calibration
        coarse_outputs = external_calibrate.external_calibrate(
            img8bit,
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_match[:num_matches],
            img_targets_match[:num_matches],
            self.vis_checker,
            self.test_config,
            max_localize_delta=None,
            reprojectionError=self.test_config["max_dist"],
        )

        # Unpack the output variables
        __, __, tgts_inliers_coarse, img_target_inliers_coarse = coarse_outputs
        rmat_coarse, tvec_coarse, __, __ = coarse_outputs

        # Get the fine-tuned external_calibration output
        incal = camera_tunnel_cal[2:]
        rmat_opt, tvec_opt = external_calibrate.external_calibrate_one_step(
            self.img,
            rmat_coarse,
            tvec_coarse,
            incal,
            self.tgts,
            self.test_config,
            self.vis_checker
        )

        # Check that the function output closely matches the expected output
        np.testing.assert_array_almost_equal(rmat_opt, self.rmat, decimal=3)
        np.testing.assert_array_almost_equal(tvec_opt, self.tvec, decimal=3)

    def test_external_calibrate_RANSAC(self):
        """
        Regression test of external_calibrate_RANSAC
        
        Outputs were manually verified to be as expected
        """
        if not test_external_calibrate_debug:
            f = open(os.devnull, 'w')
            std_stdout_orig = sys.stdout
            sys.stdout = f

        # Get image location of tgts using manual bboxes and Gaussian localization
        # Read in manual bboxes
        manual_det_filename = 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.xml'
        manual_detections_path = os.path.join(
            files_dir, 'detection', 'Annotations', manual_det_filename
        )
        
        # Get center of each image target
        img_targets = []
        for bbox in parsers.read_pascal_voc(manual_detections_path):
            # If it is a target of the appropriate class, and not flagged as difficult
            #   add it to the targets list
            if (bbox['class'] == 'dot') and not bbox['difficult']:        
                x1, y1, x2, y2 = bbox['x1'], bbox['y1'], bbox['x2'], bbox['y2']
                center = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                img_target = {'target_type' : 'dot', 
                            'center' : center}
                img_targets.append(img_target)
        
        rmat_opt, tvec_opt = external_calibrate.external_calibrate_RANSAC(
            [self.cameraMatrix, self.distCoeffs],
            self.tgts, img_targets, self.vis_checker, max_iter=0.999,
            max_dist=8, match_thresh=0.80)

        rmat_gt = [[-0.999685595,  -0.01305803,    0.0214055819],
                   [-0.0132301035,  0.9998811364, -0.0079169066],
                   [-0.0212996584, -0.0081976155, -0.9997395279]]
        tvec_gt = [[-5.0715497626], [-0.0496114238], [11.5443579906]]

        # Check that the function output closely matches the expected output
        np.testing.assert_array_almost_equal(rmat_opt, rmat_gt, decimal=2)
        np.testing.assert_array_almost_equal(tvec_opt, tvec_gt, decimal=2)

        # Redirect the std out to the original location
        if not test_external_calibrate_debug:
            sys.stdout = std_stdout_orig
            f.close()


if __name__ == "__main__":
    unittest.main()
