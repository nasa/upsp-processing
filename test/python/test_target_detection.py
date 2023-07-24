import unittest
import numpy as np
import os
import copy
import cv2

import file_structure
from upsp.target_operations import target_detection
from upsp.cam_cal_utils import (
    img_utils,
    parsers,
    photogrammetry,
    visibility,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class TargetDetectionTestCase(unittest.TestCase):
    """
    Tests for target_detection
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

    def test_subpixel_localize(self):
        """
        Regression test for subpixel_localize

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
            copy.deepcopy(self.tgts_visible),
            self.test_config
        )
        tgts_match, img_targets_match, num_matches = template_detection_output

        # Run the subpixel localization
        tgts_sub, img_targets_sub = target_detection.subpixel_localize(
            img8bit,
            tgts_match[:num_matches],
            img_targets_match,
            self.test_config,
            max_localize_delta=None
        )

        img_targets_sub_gt = [[1061.32947596, 262.49474001],
                              [1049.19571548, 602.58437063],
                              [959.21813484,  401.63819636],
                              [824.76577736,  396.81149221],
                              [806.58913148,  687.05189197],
                              [731.56243383,  407.50854921],
                              [607.99036642,  150.81565999],
                              [608.23270776,  399.98498769],
                              [366.61396903,  538.20378513],
                              [206.82895966,  212.73496326],
                              [114.70997899,  397.03944153]]

        for i in range(len(img_targets_sub)):
            np.testing.assert_array_almost_equal(
                img_targets_sub[i]['center'],
                img_targets_sub_gt[i],
                decimal=2,
            )

    # pixel_2_polygon is tested indirectly with the test_template_detection

    # area_overlap is tested indirectly with the test_template_detection

    def test_template_detection(self):
        """
        Regression test for template_detection

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
            copy.deepcopy(self.tgts_visible),
            self.test_config
        )
        tgts_match, img_targets_match, num_matches = template_detection_output
        self.assertEqual(num_matches, 11)

        img_targets_match_gt = [[1061.5, 262.5],
                                [1049.5, 603.5],
                                [959.5,  402.5],
                                [825.,   397.5],
                                [806.5,  688. ],
                                [732.,   408. ],
                                [609.,   152. ],
                                [609.,   400.5],
                                [367.,   539. ],
                                [207.5,  213.5],
                                [116.,   398. ]]
        
        for i in range(num_matches):
            np.testing.assert_array_almost_equal(
                img_targets_match_gt[i], img_targets_match[i]['center']
            )

    def test_filter_dist_filter(self):
        """
        Unit test of filter_dist_filter for basic cases
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
            copy.deepcopy(self.tgts_visible),
            self.test_config
        )
        tgts_match, img_targets_match, num_matches = template_detection_output

        # Check 1
        tgts_test = copy.deepcopy(tgts_match)
        tgt_fake = copy.deepcopy(tgts_test[num_matches-1])
        tgt_fake['tvec'][0] += 0.1
        tgts_test.append(tgt_fake)
        tgts_det, img_targets, num_matches_filter = target_detection.filter_dist_filter(
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_test,
            img_targets_match,
            num_matches,
            8,
            6
        )
        self.assertEqual(num_matches_filter, num_matches - 1)

        # Check 2
        img_targets_test = copy.deepcopy(img_targets_match)
        img_target_fake = copy.deepcopy(img_targets_test[num_matches-1])
        img_target_fake['center'][0] += 2
        img_target_fake['center'][1] += 2
        img_targets_test.append(img_target_fake)
        tgts_det, img_targets, num_matches_filter = target_detection.filter_dist_filter(
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_match,
            img_targets_test,
            num_matches,
            8,
            6
        )
        self.assertEqual(num_matches_filter, num_matches - 1)

        # Check 3
        img_targets_test = copy.deepcopy(img_targets_match)
        img_target_fake = copy.deepcopy(img_targets_test[-1])
        img_target_fake['center'] = 113.5, 401.5
        img_targets_test.append(img_target_fake)
        tgts_det, img_targets, num_matches_filter = target_detection.filter_dist_filter(
            rmat,
            tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_match,
            img_targets_test,
            num_matches,
            8,
            6
        )
        self.assertEqual(num_matches_filter, num_matches - 1)

    # filter_partially_occluded is not yet implemented in the filtering pipeline, so it
    #   doesn't have to be tested yet

    # filter_matches at the moment only implements filter_dist_filter, so if
    #   filter_dist_filter passes, filter_matches will pass as well

if __name__ == "__main__":
    unittest.main()
