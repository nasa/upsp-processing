import unittest
import numpy as np
import os

import file_structure
from upsp.cam_cal_utils import parsers

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class ParsersTestCase(unittest.TestCase):
    """
    Unit tests for the parsers module
    """
    
    def setUp(self):
        """
        Checks files set up properly before each running test
        """
        
        file_structure.setUp(files_dir)

    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()

    def test_read_tgts(self):
        """
        Unit test for read_tgts
        """
        # Read in the test tgts file
        tgts_path = os.path.join(files_dir, 'fml_tc3_volume.tgts')

        # Spot check on a couple of items with different, valid inputs
        tgts = parsers.read_tgts(tgts_path, 'dot')
        np.testing.assert_array_almost_equal(
            tgts[6]['tvec'], np.array([[-7.4388], [-0.145], [-7.0]]), decimal=4
        )
        np.testing.assert_array_equal(tgts[6]['tvec'].shape, (3, 1))

        tgts = parsers.read_tgts(tgts_path, ['dot'])
        np.testing.assert_array_almost_equal(
            tgts[6]['tvec'], np.array([[-7.4388], [-0.145], [-7.0]]), decimal=4
        )
        
    def test_read_pascal_voc(self):
        """
        Unit test for read_pascal_voc
        """
        # Define the filepath and the ground truth results
        pv_path =  os.path.join(
            files_dir,
            'detection',
            'Annotations',
            'CAM1_RUN8_CINE02_Y20000209H11294501.00001.xml'
        )
        pv_gt_1 = {
            'class': 'dot',
            'x1': 605,
            'x2': 612,
            'y1': 397,
            'y2': 404,
            'difficult': 0,
            'truncated': 0
        }
        pv_gt_2 = {
            'class': 'dot',
            'x1': 804,
            'x2': 810,
            'y1': 685,
            'y2': 691,
            'difficult': 0,
            'truncated': 0
        }
        
        # Tests capability to read list of annots
        pv = parsers.read_pascal_voc([pv_path, pv_path])
        
        self.assertEqual(pv[0][4], pv_gt_1)
        self.assertEqual(pv[0][11], pv_gt_2)

        # Tests capability to read single annot
        pv = parsers.read_pascal_voc(pv_path)
        self.assertEqual(pv[4], pv_gt_1)
        self.assertEqual(pv[11], pv_gt_2)

    def test_read_wind_tunnel_data(self):
        """
        Unit test for read_wind_tunnel_data
        """
        # Read the wind tunnel data file then compare to the baseline
        wtd = parsers.read_wind_tunnel_data(os.path.join(files_dir, 'wtd_test.wtd'))
        self.assertEqual(
            wtd, {'ALPHA': 0.05, 'BETA': 0.12, 'PHI': 0.90, 'STRUTZ': 10.0}
        )

    def test_convert_cv2_cm_to_uPSP_cm(self):
        """
        Unit test for convert_cv2_cm_to_uPSP_cm
        """
        # Define a traditional camera matrix (with realistic parameters)
        cm = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                       [0.0, 1380.2632820187425, 256.778541140320840], 
                       [0.0, 0.0, 1.0]])
        
        # Convert to a uPSP camera matrix
        uPSP_cm = parsers.convert_cv2_cm_to_uPSP_cm(cm, (512, 1024))

        # Define the ground truth for the uPSP camera matrix
        uPSP_cm_gt = np.array([[1380.2632820187425, 0.0, 21.908701486902032],
                               [0.0, 1380.2632820187425, 0.778541140320840], 
                               [0.0, 0.0, 1.0]])

        # Compare the parsers' converted matrix to the ground truth
        np.testing.assert_array_almost_equal(uPSP_cm, uPSP_cm_gt, decimal=4)
        np.testing.assert_array_equal(uPSP_cm.shape, (3, 3))

    def test_convert_uPSP_cm_to_cv2_cm(self):
        """
        Unit test for convert_uPSP_cm_to_cv2_cm
        """
        # Define a uPSP camera matrix (with realistic parameters)
        uPSP_cm = np.array([[1380.2632820187425, 0.0, 21.908701486902032],
                            [0.0, 1380.2632820187425, 0.778541140320840], 
                            [0.0, 0.0, 1.0]])
        
        # Convert to a traditional camera matrix
        cm = parsers.convert_uPSP_cm_to_cv2_cm(uPSP_cm, (512, 1024))
        
        # Define the ground truth for the camera matrix
        cm_gt = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                          [0.0, 1380.2632820187425, 256.778541140320840], 
                          [0.0, 0.0, 1.0]])
        
        # Compare the parsers' converted uPSP matrix to the ground truth
        np.testing.assert_array_almost_equal(cm, cm_gt, decimal=4)
        np.testing.assert_array_equal(cm.shape, (3, 3))

    def test_read_internal_params(self):
        """
        Unit test for read_internal_params
        """
        # Read the given intrinsics file
        intrinsics_path = os.path.join(
            files_dir, 'internal-calibration', 'camera01_35_6.json'
        )

        cm, dists, sr, ss = parsers.read_internal_params(
            intrinsics_path, (512, 1024), read_sensor=True
        )
        
        # Compare the ground truth camera matrix to the parsed matrix
        cm_gt = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                          [0.0, 1380.2632820187425, 256.778541140320840], 
                          [0.0, 0.0, 1.0]])
        np.testing.assert_array_almost_equal(cm, cm_gt, decimal=4)
        np.testing.assert_array_equal(cm.shape, (3, 3))

        # Compare the ground truth distortion coefficients to the parsed coefficients
        dists_gt = np.array([[-0.09098491035825468, 0.0, 0.0, 0.0, 0.0]])
        np.testing.assert_array_almost_equal(dists, dists_gt, decimal=4)
        np.testing.assert_array_equal(dists.shape, (1, 5))

        # Compare sensor resolutoin and sensor size
        sensor_resolution_gt = np.array([800, 1280, 1])
        np.testing.assert_array_almost_equal(sr, sensor_resolution_gt, decimal=4)

        sensor_size_gt = np.array([0.8818898, 1.4110236])
        np.testing.assert_array_almost_equal(ss, sensor_size_gt, decimal=4)

    def test_read_camera_tunnel_cal(self):
        """
        Unit test for read_camera_tunnel_cal
        """
        # Read in the camera calibration
        camera_tunnel_cal_path = os.path.join(
            files_dir, 'camera-tunnel-calibration', 'camera01_35_6.json'
        )
        rmat, tvec, cm, dists, sr, ss = parsers.read_camera_tunnel_cal(
            camera_tunnel_cal_path, (512, 1024), read_sensor=True
        )

        # Compare the ground truth camera matrix to the parsed matrix
        cm_gt = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                          [0.0, 1380.2632820187425, 256.778541140320840], 
                          [0.0, 0.0, 1.0]])
        np.testing.assert_array_almost_equal(cm, cm_gt, decimal=1)
        np.testing.assert_array_equal(cm.shape, (3, 3))

        # Compare the ground truth distortion coefficients to the parsed coefficients
        dists_gt = np.array([[-0.09098491035825468, 0.0, 0.0, 0.0, 0.0]])
        np.testing.assert_array_almost_equal(dists, dists_gt, decimal=1)
        np.testing.assert_array_equal(dists.shape, (1, 5))

        # Compare the ground truth rotation matrix to the parsed matrix
        rmat_gt = np.array([[-0.9997264805692, -0.01297871345068,  0.01945551453603],
                            [-0.0131837243006,  0.99985852053613, -0.01044645034786],
                            [-0.0193171804946, -0.01070008918048, -0.99975614758263]])
        np.testing.assert_array_almost_equal(rmat, rmat_gt, decimal=1)
        np.testing.assert_array_equal(rmat.shape, (3, 3))

        # Compare the ground truth translation vector to the parsed vector
        tvec_gt = np.expand_dims([-5.09303598682, -0.0771666696565, 11.55605419793], 1)
        np.testing.assert_array_almost_equal(tvec, tvec_gt, decimal=1)
        np.testing.assert_array_equal(tvec.shape, (3, 1))

        # Compare sensor resolutoin and sensor size
        sensor_resolution_gt = np.array([800, 1280, 1])
        np.testing.assert_array_almost_equal(sr, sensor_resolution_gt, decimal=4)

        sensor_size_gt = np.array([0.8818898, 1.4110236])
        np.testing.assert_array_almost_equal(ss, sensor_size_gt, decimal=4)

    def test_read_json(self):
        """
        Unit test for read_json
        """
        # Read in the test config as a regular json
        tc = parsers.read_json(os.path.join(files_dir, 'test_config.json'))

        # Assert that Updated is an item in the test_config file
        # Remove it since the cached version might be more updated than the read version
        assert "Updated" in tc
        del tc["Updated"]

        # Define the ground truth
        tc_gt = {"oblique_angle" : 70,
                "tunnel-cor_to_tgts_tvec" : np.zeros(3),
                "tunnel-cor_to_tgts_rmat": np.eye(3),
                "tunnel-cor_to_tunnel-origin_tvec": np.zeros(3),
                "tunnel-cor_to_tunnel-origin_rmat": np.eye(3),
                "crosscorr_coeff" : 0.75,
                "dot_pad" : 4,
                "kulite_pad" : 3,
                "max_dist" : 6,
                "min_dist" : 8} 
        
        # Compare the parsed to the ground truth
        for key in tc_gt.keys():
            np.testing.assert_array_equal(tc[key], tc_gt[key])
        for key in tc.keys():
            np.testing.assert_array_equal(tc[key], tc_gt[key])

    def test_read_test_config(self):
        """
        Unit test for read_test_config
        """
        # Read in the test config file
        tc = parsers.read_test_config(os.path.join(files_dir, 'test_config.json'))

        # Assert that Updated is an item in the test_config file
        # Remove it since the cached version might be more updated than the read version
        assert "Updated" in tc
        del tc["Updated"]

        # Define the ground truth
        tc_gt = {"oblique_angle" : 70,
                "tunnel-cor_to_tgts_tvec" : np.zeros((3, 1)),
                "tunnel-cor_to_tgts_rmat": np.eye(3),
                "tunnel-cor_to_tunnel-origin_tvec": np.zeros((3, 1)),
                "tunnel-cor_to_tunnel-origin_rmat": np.eye(3),
                "crosscorr_coeff" : 0.75,
                "dot_pad" : 4,
                "kulite_pad" : 3,
                "max_dist" : 6,
                "min_dist" : 8} 
        
        # Compare the parsed to the ground truth
        for key in tc_gt.keys():
            np.testing.assert_array_equal(tc[key], tc_gt[key])
        for key in tc.keys():
            np.testing.assert_array_equal(tc[key], tc_gt[key])

        np.testing.assert_array_equal(tc['tunnel-cor_to_tgts_tvec'].shape, (3, 1))
        np.testing.assert_array_equal(tc['tunnel-cor_to_tgts_rmat'].shape, (3, 3))
        np.testing.assert_array_equal(
            tc['tunnel-cor_to_tunnel-origin_tvec'].shape, (3, 1)
        )
        np.testing.assert_array_equal(
            tc['tunnel-cor_to_tunnel-origin_rmat'].shape, (3, 3)
        )


if __name__ == "__main__":
    unittest.main()
