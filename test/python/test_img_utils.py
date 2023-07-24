import unittest
import numpy as np
import os
import cv2

from upsp.cam_cal_utils import img_utils

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class ImgUtilsTestCase(unittest.TestCase):
    """
    Tests for the img_utils module
    """
    
    def test_convert_12bit_to_8bit(self):
        """
        Regression tests convert_12bit_to_8bit

        Outputs were manually verified to be as expected
        """

        # Tests that the output is being correctly scaled
        inp = np.array([0, 4095, 4078, 128, 369, 3184], dtype=np.uint16)
        out_true = np.array([0, 255, 254, 8, 23, 198], dtype=np.uint8)
        out_func = img_utils.convert_12bit_to_8bit(inp)
        np.testing.assert_array_equal(out_func, out_true)

        # Test for when 1 output is incorrectly scaled
        inp = np.array([0, 4095, 4078, 128, 369, 3184], dtype=np.uint16)
        out_true = np.array([1, 255, 254, 8, 23, 198], dtype=np.uint8)
        out_func = img_utils.convert_12bit_to_8bit(inp)
        np.testing.assert_equal(np.any(np.not_equal(out_true, out_func)), True)

        # Test for when all outputs are incorrectly scaled
        inp = np.array([0, 4095, 4078, 128, 369, 3184], dtype=np.uint16)
        out_true = np.array([42, 42, 42, 42, 42, 42], dtype=np.uint8)
        out_func = img_utils.convert_12bit_to_8bit(inp)
        np.testing.assert_equal(np.any(np.not_equal(out_true, out_func)), True)

    # scale_image is tested via test_convert_12bit_to_8bit and
    #   test_scale_image_max_inlier

    def test_scale_image_max_inlier(self):
        """
        Regression tests scale_image_max_inlier

        Outputs were manually verified to be as expected
        """

        # Test for when the 2nd greatest value is within 98% of the 1st greatest value
        #   No change from convert_12bit_to_8bit
        inp = np.array([0, 4095, 4078, 128, 369, 3184], dtype=np.uint16)
        out_true = np.array([0, 255, 254, 8, 23, 198], dtype=np.uint8)
        out_func = img_utils.scale_image_max_inlier(inp)
        np.testing.assert_array_equal(out_func, out_true)

        # Test for when 2nd greatest value is NOT within 98% of 1st greatest value, but
        #   3rd greatest value is within 98% of 2nd greatest value
        inp = np.array([0, 4095, 2000, 1999, 1002, 918], dtype=np.uint16)
        out_true = np.array([0, 255, 255, 255, 128, 117], dtype=np.uint8)
        out_func = img_utils.scale_image_max_inlier(inp)
        np.testing.assert_array_equal(out_func, out_true)

    def test_interp(self):
        """
        Regression test of interp

        Outputs were manually verified to be as expected
        """
        img_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )

        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH)
        pts = np.array([[1.4, 1.2], [500.3, 254.1], [741.9, 452.8], [1011.0, 400.]])

        res = img_utils.interp(pts, img, method='nearest')
        np.testing.assert_equal(res, [2., 133., 8., 118.])
        
        res = img_utils.interp(pts, img, method='linear')
        np.testing.assert_almost_equal(res, [1.92, 133.09, 7.72, 118.])
        

if __name__ == "__main__":
    unittest.main()
