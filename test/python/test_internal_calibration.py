import unittest
import numpy as np
import os

import file_structure
from upsp.cam_cal_utils import (
    internal_calibration,
    parsers,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class InternalCalibrationTestCase(unittest.TestCase):
    """
    Tests for the img_utils module
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

        # Set the debug boolean to False
        internal_calibration.debug_show_cal_bounds = False
     
    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """
        # We can rerun setUp because the file structure should be unchanged
        self.setUp()

    def test_get_pts_inside_incal(self):   
        """
        Regression test for get_pts_inside_incal

        Outputs were manually verified to be as expected
        """
        area_debug, vol_debug = internal_calibration.incal_calibration_bounds_debug()
        obj_pts = []
        max_val = 50
        for x in range(max_val+1):
            for y in range(max_val+1):
                obj_pts.append((x, y, 10.))
        obj_pts = np.array(obj_pts)
        obj_pts -= (max_val/2, max_val/2, 0.)

        safe_obj_idxs = internal_calibration.get_pts_inside_incal(
            np.eye(3), np.zeros((3, 1)), np.eye(3), np.array([[-0.05, 0., 0., 0., 0.]]),
            obj_pts, area_debug, vol_debug, critical_pt = 'first')
        safe_obj_pts = set([tuple(obj_pts[i].tolist()) for i in safe_obj_idxs])
        
        safe_obj_pts_gt = [
            ( 22.0,  11.0, 10.0), (-5.0,  2.0, 10.0), ( 25.0, -7.0, 10.0),
            (-17.0,   0.0, 10.0), (-3.0, -9.0, 10.0), (-17.0, -3.0, 10.0),
            ( -8.0,  -7.0, 10.0), (-4.0,  8.0, 10.0), (-10.0, 23.0, 10.0),
            ( -3.0, -22.0, 10.0), (10.0, 14.0, 10.0), (-12.0, -6.0, 10.0)
        ]

        not_safe_obj_pts_gt = [
            (25.0,  16.0, 10.0), ( 23.0, -23.0, 10.0), (-25.0,  20.0, 10.0),
            (17.0,  24.0, 10.0), (-22.0, -24.0, 10.0), ( 25.0,  13.0, 10.0),
            (-15.0, 25.0, 10.0), (-21.0,  24.0, 10.0), (-22.0,  25.0, 10.0),
            (20.0,  25.0, 10.0), (-23.0,  21.0, 10.0), (-25.0, -13.0, 10.0)
        ]

        self.assertEqual(len(safe_obj_pts), 2413)
        for pt in safe_obj_pts_gt:
            self.assertTrue(pt in safe_obj_pts)
        for pt in not_safe_obj_pts_gt:
            self.assertTrue(pt not in safe_obj_pts)
    
    # incal_from_calibio is tested via write_incal_from_calibio
    
    def test_write_incal_from_calibio(self):
        """
        Unit test for write_incal_from_calibio
        """
        calibio_path = os.path.join(
            files_dir, 'internal-calibration', 'calibio_incal_test.json'
        )
        
        output_dir = os.path.join(files_dir, 'output')
        internal_calibration.write_incal_from_calibio(
            calibio_path, 'written_camera', [0.8, 1.2], output_dir
        )
        incal_output_dir = os.path.join(output_dir, 'written_camera.json')

        incal_output = parsers.read_internal_params(
            incal_output_dir, dims=(1000, 1000), read_sensor=True
        )
        cameraMatrix_wi, distCoeffs_wi, img_size_wi, sensor_sise_wi = incal_output
        
        cameraMatrix_gt = [[2.0037e+03, 0.0000e+00, 5.1452e+02],
                           [0.0000e+00, 2.0037e+03, 4.8666e+02],
                           [0.0000e+00, 0.0000e+00, 1.0000e+00]]
        distCoeffs_gt   = [[-0.174,   0.1268,  0.,      0.,      0.    ]]
        img_size_gt     = [ 800, 1280]
        sensor_sise_gt  = [0.8, 1.2]

        np.testing.assert_array_almost_equal(
            cameraMatrix_gt, cameraMatrix_wi, decimal=2
        )
        np.testing.assert_array_almost_equal(distCoeffs_gt, distCoeffs_wi, decimal=2)
        np.testing.assert_array_almost_equal(img_size_gt, img_size_wi, decimal=2)
        np.testing.assert_array_almost_equal(sensor_sise_gt, sensor_sise_wi, decimal=2)

        # Deleted the generated internal calibration json file
        os.remove(incal_output_dir)

    def test_uncertainties_from_calibio(self):
        """
        Unit test for uncertainties_from_calibio
        """
        calibio_path = os.path.join(
            files_dir, 'internal-calibration', 'calibio_incal_test.json'
        )
        uncertainties = internal_calibration.uncertainties_from_calibio(calibio_path)
        uncertainties_gt = [1.5056e+00,8.2730e-01, 9.8265e-01, 9.1580e-04, 4.0277e-03,
                            0.0000e+00, 0.0000e+00, 0.0000e+00]
        np.testing.assert_array_almost_equal(
            uncertainties, uncertainties_gt, decimal=4
        )

    def test_incal_calibration_bounds(self):
        """
        Regression test for incal_calibration_bounds

        Outputs were manually verified to be as expected
        """
        calibio_path = os.path.join(
            files_dir, 'internal-calibration', 'calibio_incal_test.json'
        )
        cal_is_safes = internal_calibration.incal_calibration_bounds(
            calibio_path, 0.075, 0.025
        )
        cal_area_is_safe, cal_vol_is_safe = cal_is_safes

        incal_from_calib = internal_calibration.incal_from_calibio(calibio_path)
        img_size, uPSP_cameraMatrix, distCoeffs = incal_from_calib
        cameraMatrix = parsers.convert_uPSP_cm_to_cv2_cm(uPSP_cameraMatrix, img_size)
        
        pixels = np.array(list(np.ndindex(img_size[1], img_size[0])))
        pixel_bools = cal_area_is_safe(pixels)
        unsafe_points = np.array(pixels[np.where(~pixel_bools)[0]])
        safe_points = np.array(pixels[np.where(pixel_bools)[0]])

        self.assertEqual(len(safe_points), 904491)
        self.assertEqual(len(unsafe_points), 119509)

        z = 95.0
        xs = (pixels[:, 0] - cameraMatrix[0][2]) / cameraMatrix[0][0] * z
        ys = (pixels[:, 1] - cameraMatrix[1][2]) / cameraMatrix[0][0] * z
        zs = np.full(xs.shape, z)
        pts = np.array([xs, ys, zs]).T
        
        pt_bools = cal_vol_is_safe(pts)
        unsafe_points = np.array(pixels[np.where(~pt_bools)[0]])
        safe_points = np.array(pixels[np.where(pt_bools)[0]])
        
        self.assertEqual(len(safe_points), 786715)
        self.assertEqual(len(unsafe_points), 237285)

    def test_incal_calibration_bounds_debug(self):
        """
        Unit test for incal_calibration_bounds_debug
        """
        is_safe_debugs = internal_calibration.incal_calibration_bounds_debug()
        cal_area_is_safe, cal_vol_is_safe = is_safe_debugs
        img_size = (512, 1024)
        cameraMatrix = np.array([[650, 0, 256], [650, 0, 512], [0, 0, 1]], dtype=float)
        
        pixels = np.array(list(np.ndindex(img_size[1], img_size[0])))
        pixel_bools = cal_area_is_safe(pixels)
        unsafe_points = np.array(pixels[np.where(~pixel_bools)[0]])
        safe_points = np.array(pixels[np.where(pixel_bools)[0]])

        self.assertEqual(len(safe_points), 524288)
        self.assertEqual(len(unsafe_points), 0)

        z = 95.0
        xs = (pixels[:, 0] - cameraMatrix[0][2]) / cameraMatrix[0][0] * z
        ys = (pixels[:, 1] - cameraMatrix[1][2]) / cameraMatrix[0][0] * z
        zs = np.full(xs.shape, z)
        pts = np.array([xs, ys, zs]).T
        
        pt_bools = cal_vol_is_safe(pts)
        unsafe_points = np.array(pixels[np.where(~pt_bools)[0]])
        safe_points = np.array(pixels[np.where(pt_bools)[0]])

        self.assertEqual(len(safe_points), 524288)
        self.assertEqual(len(unsafe_points), 0)        
        
    # alpha_shape_is_safe is tested via incal_calibration_bounds


if __name__ == "__main__":
    unittest.main()
