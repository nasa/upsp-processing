import unittest
import numpy as np
import os
import copy

import file_structure
from upsp.cam_cal_utils import (
    camera_tunnel_calibrate,
    parsers,
    photogrammetry,
    visibility,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class PhotogrammetryTestCase(unittest.TestCase):
    """
    Tests for photogrammetry
    """

    @classmethod
    def setUpClass(cls):       
        # Define the rotation matrix and translation vector
        cls.rmat = np.array([[-0.999726480569, -0.0129787134506,  0.0194555145360],
                             [-0.013183724300,  0.9998585205361, -0.0104464503478],
                             [-0.019317180494, -0.0107000891804, -0.9997561475826]])
        cls.tvec = np.array([[-5.093035986816], [-0.07716666965650], [11.556054197934]])
        
        # Define the camera matrix and distortion coefficients
        cls.cameraMatrix = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                                     [0.0, 1380.2632820187425, 256.778541140320840], 
                                     [0.0, 0.0, 1.0]])
        cls.distCoeffs = np.array([[-0.09098491035825468, 0.0, 0.0, 0.0, 0.0]])

        # Read the rest configuration file
        test_config_file = os.path.join(files_dir, 'test_config.json')
        test_config = parsers.read_json(test_config_file)

        # Get the visibility checker from the grid
        grd_file = os.path.join(files_dir, 'fml_tc3_volume.grid')
        cls.vis_checker = visibility.VisibilityChecker(grd_file,
            test_config['oblique_angle'], epsilon=1e-4
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
        Checks files set up properly before each running test
        """
        
        file_structure.setUp(files_dir)

    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()

    def test_rot(self):
        """
        Regression test for rot

        Randomly generated inputs tests against a version of the function cached on
            03/11/2021. Outputs of function at time were manually tested and verified
            to be correct
        """
        
        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)
        
        # This is a cached version of the function. Should the function be changed in
        #   the future, we want to ensure consistency between the old and new function
        def regression_rot(angle, axis):
            angle_rad = np.deg2rad(angle)

            if axis == 'x':
                return np.array([[1,                 0,                  0],
                                [0, np.cos(angle_rad), -np.sin(angle_rad)],
                                [0, np.sin(angle_rad),  np.cos(angle_rad)]])

            if axis == 'y':
                return np.array([[np.cos(angle_rad), 0,   np.sin(angle_rad)],
                                [0,                  1,                   0],
                                [-np.sin(angle_rad), 0,   np.cos(angle_rad)]])

            if axis == 'z':
                return np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                                [np.sin(angle_rad),  np.cos(angle_rad), 0],
                                [0,                  0,                 1]])

        # Generate 50 random angles and axes
        random_angles = (360 * rng.random(50)) - 180
        axes = rng.choice(['x', 'y', 'z'], 50, replace=True)
        
        # Test photogrammetry.rot against the cached version
        for angle, axis in zip(random_angles, axes):
            p_rot = photogrammetry.rot(angle, axis)
            r_rot = regression_rot(angle, axis)
            np.testing.assert_array_almost_equal(p_rot, r_rot, decimal=3)
            np.testing.assert_array_almost_equal(p_rot.shape, (3, 3))

    def test_invTransform(self):
        """
        Unit test for invTransform

        Many randomly generated inputs tests known properties of the outputs
        """

        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)

        # Test 50 times (50 random inputs)
        for i in range(50):
            # Generate 3 random rotation matrices
            angles = (360 * rng.random(3)) - 180
            rotx = photogrammetry.rot(angles[0], 'x')
            roty = photogrammetry.rot(angles[1], 'y')
            rotz = photogrammetry.rot(angles[2], 'z')

            # Combine the 3 random rotation matrices with a random translation vector
            rmat = np.matmul(rotx, roty, rotz)
            tvec = (360 * rng.random((3, 1))) - 180
            
            # Get the inverse of the rotation and translation
            inv_rmat, inv_tvec = photogrammetry.invTransform(rmat, tvec)

            # To test the inverse rotation matrix, use the fact that RR' = I
            #   Allow for machine precision errors
            near_eye_rmat = np.matmul(rmat, inv_rmat)
            np.testing.assert_array_almost_equal(near_eye_rmat, np.eye(3), decimal=3)
            np.testing.assert_array_almost_equal(near_eye_rmat.shape, (3, 3))

            # To test the translation vector, use the fact that Rt' + t = 0 
            #   Allow for machine precision errors
            near_zero_tvec = tvec + np.matmul(rmat, inv_tvec)
            np.testing.assert_array_almost_equal(
                near_zero_tvec, np.zeros((3, 1)), decimal=3
            )
            np.testing.assert_array_almost_equal(near_zero_tvec.shape, (3, 1))

    def test_isRotationMatrix(self):
        """
        Unit test for isRotationMatrix
        
        Many randomly generated inputs tests known properties of the outputs
        """
        
        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)

        # Test 50 times
        for i in range(50):
            # Generate a random but valid rotation matrix
            angles = (360 * rng.random(3)) - 180
            rotx = photogrammetry.rot(angles[0], 'x')
            roty = photogrammetry.rot(angles[1], 'y')
            rotz = photogrammetry.rot(angles[2], 'z')
            rmat = np.matmul(rotz, roty, rotx)

            # Test if the rotation matrix is valid
            self.assertEqual(photogrammetry.isRotationMatrix(rmat), True)
        
        # Test against an obviously invalid rotation matrix
        rmat = np.zeros((3, 3))
        self.assertEqual(photogrammetry.isRotationMatrix(rmat), False)
        
        # Test against an obviously invalid rotation matrix
        rmat = np.full((3, 3), 1)
        self.assertEqual(photogrammetry.isRotationMatrix(rmat), False)
    
    def test_isRotationMatrixRightHanded(self):
        """
        Unit test for isRotationMatrixRightHanded

        Begin with a left handed and right handed matrix. Perform random rotations
        of those matrixes. Rotated left handed should always be not-right-handed and
        rotated right handed should always be right handed
        """
        righty = np.eye(3)
        lefty = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])
        
        rng = np.random.default_rng(0)
        randoms = (360 * rng.random((50, 3))) - 180
        for random in randoms:
            righty_i = np.matmul(righty, photogrammetry.rot(random[0], 'x'))
            righty_i = np.matmul(righty_i, photogrammetry.rot(random[1], 'y'))
            righty_i = np.matmul(righty_i, photogrammetry.rot(random[2], 'z'))
            
            lefty_i = np.matmul(lefty, photogrammetry.rot(random[0], 'x'))
            lefty_i = np.matmul(lefty_i, photogrammetry.rot(random[1], 'y'))
            lefty_i = np.matmul(lefty_i, photogrammetry.rot(random[2], 'z'))
            
            self.assertEqual(
                photogrammetry.isRotationMatrixRightHanded(righty_i), True
            )
            self.assertEqual(
                photogrammetry.isRotationMatrixRightHanded(lefty_i), False
            )

    def test_rotationMatrixToTunnelAngles(self):
        """
        Unit test for rotationMatrixToTunnelAngles
        
        Many randomly generated inputs tests known properties of the outputs
        """
        
        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)

        # Test with 50 random inputs
        for i in range(50):
            angles = (179.998 * rng.random(3)) - 89.999

            # Generate the rotation matrix
            rmat, tvec = camera_tunnel_calibrate.tunnel_transform(
                angles[0], angles[1], angles[2], 0, [0, 0, 0]
            )

            # Convert rotation matrix to euler angles
            angles_calculated = photogrammetry.rotationMatrixToTunnelAngles(rmat)

            # Test that the rotation matrix converted from euler angles is the same as
            #    the original rotation matrix
            np.testing.assert_array_almost_equal(
                np.expand_dims(angles, 1), angles_calculated, decimal=3
            )
    
    def test_transform_3d_point(self):
        """
        Regression test for transform_3d_point

        Tests a subset of the targets for projected pixel location and pixel jacobian
        """ 
        obj_pts = np.squeeze([tgt['tvec'] for tgt in self.tgts_visible], axis=2)

        # Get the transformed targets
        transforms = photogrammetry.transform_3d_point(self.rmat, self.tvec, obj_pts)

        # Check the transformed targets are as expected
        self.assertEqual(transforms.shape, (24, 3))
        np.testing.assert_array_almost_equal(
            transforms[0], [5.49454348, -1.89638399, 18.782807], decimal=3
        )
        np.testing.assert_array_almost_equal(
            transforms[1], [5.30912805,  2.76660932, 18.730475], decimal=3
        )
        np.testing.assert_array_almost_equal(
            transforms[2], [4.04636575,  0.01597572, 18.7346435], decimal=3
        )

    def test_project_3d_point(self):
        """
        Regression test for project_3d_point

        Tests a subset of the targets for projected pixel location and pixel jacobian
        """        
        obj_pts = np.array([tgt['tvec'] for tgt in self.tgts])
        projs = photogrammetry.project_3d_point(
            self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs, obj_pts
        )

        # Check that projections are correct
        np.testing.assert_array_equal(projs.shape, (24, 2))

        np.testing.assert_array_almost_equal(projs[0],  [934.159, 118.636], decimal=2)
        np.testing.assert_array_almost_equal(projs[2],  [830.756, 257.950], decimal=2)
        np.testing.assert_array_almost_equal(projs[4],  [768.440, 396.979], decimal=2)
        np.testing.assert_array_almost_equal(projs[7],  [678.448, 542.530], decimal=2)
        np.testing.assert_array_almost_equal(projs[11], [481.248, 445.495], decimal=2)
        np.testing.assert_array_almost_equal(projs[17], [-128.84, 184.490], decimal=2)
        np.testing.assert_array_almost_equal(projs[23], [-564.553, 413.039], decimal=2)
        np.testing.assert_array_almost_equal(projs[-1], [-564.553, 413.039], decimal=2)

        projs, jac = photogrammetry.project_3d_point(
            self.rmat,
            self.tvec,
            self.cameraMatrix,
            self.distCoeffs,
            obj_pts,
            ret_jac=True
        )

        jac7 = np.array([[ 1.81736046e+02,  5.78473766e+02, -2.05491850e+01,
                           7.34788956e+01, -2.94818066e-01, -7.67152670e+00],
                         [-3.35503179e+02,  1.01839467e+02, -3.67421363e+02,
                          -2.94818066e-01,  7.30451713e+01, -1.51664631e+01]])

        jac19 = np.array([[-96.40308249, 719.56242991,  -48.06341937,
                            66.9415494,   -0.936783,     41.16804995],
                          [287.24119638,  46.27631763, -330.32092328,
                            -0.936783,    71.94503581,    7.45497706]])
        
        np.testing.assert_array_equal(jac.shape, (24, 2, 6))
        np.testing.assert_array_almost_equal(jac[7],  jac7,  decimal=2)
        np.testing.assert_array_almost_equal(jac[19], jac19, decimal=2)

    def test_transform_targets(self):
        """
        Regression test for transform_targets
        
        The solutions were manually verified visually with the debug outputs
        """

        # Define the rotation matrix and translation vector
        rmat = np.array([[-0.99972648056921, -0.012978713450675,  0.019455514536029],
                         [-0.01318372430061,  0.999858520536128, -0.010446450347859],
                         [-0.01931718049462, -0.010700089180476, -0.999756147582633]])
        tvec = np.array([[-5.0930359869], [-0.077166669657], [11.5560541979]])
        
        # Read in the targets file
        tgts_file = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        tgts = parsers.read_tgts(tgts_file)

        cameraMatrix = np.array([[1380.2632820187425, 0.0, 533.908701486902032],
                                 [0.0, 1380.2632820187425, 256.778541140320840], 
                                 [0.0, 0.0, 1.0]])
        distCoeffs = np.array([[-0.09098491035825468, 0.0, 0.0, 0.0, 0.0]])

        tgts = photogrammetry.get_visible_targets(
            rmat,
            tvec,
            cameraMatrix,
            distCoeffs,
            tgts,
            self.vis_checker
        )

        # Get the transformed targets
        transforms = photogrammetry.transform_targets(rmat, tvec, tgts)

        tvec0 = np.array([[5.49454348], [-1.89638399], [18.78280763]])
        norm0 = np.array([[ 0.01945551], [-0.01044645], [-0.99975615]])
        
        tvec1 = np.array([[5.30912805], [2.76660932], [18.73047579]])
        norm1 = np.array([[0.01945551], [-0.01044645], [-0.99975615]])

        tvec2 = np.array([[4.04636575], [0.0159757297], [18.7346435]])
        norm2 = np.array([[0.01945551], [-0.01044645], [-0.99975615]])

        # Check the transformed targets are as expected
        np.testing.assert_array_almost_equal(transforms[0]['tvec'], tvec0, decimal=3)
        np.testing.assert_array_almost_equal(transforms[0]['norm'], norm0, decimal=3)
        
        np.testing.assert_array_almost_equal(transforms[1]['tvec'], tvec1, decimal=3)
        np.testing.assert_array_almost_equal(transforms[1]['norm'], norm1, decimal=3)
         
        np.testing.assert_array_almost_equal(transforms[2]['tvec'], tvec2, decimal=3)
        np.testing.assert_array_almost_equal(transforms[2]['norm'], norm2, decimal=3)
                
    def test_project_targets(self):
        """
        Regression test for project_targets
        
        The solutions were manually verified visually with the debug outputs
        """
        # Project the targets into the image
        projs = photogrammetry.project_targets(
            self.rmat, self.tvec, self.cameraMatrix, self.distCoeffs, self.tgts_visible)
        
        # Test the target projections are as expected
        self.assertEqual(len(projs), len(self.tgts_visible))
        np.testing.assert_array_almost_equal(
            projs[0]['proj'], [934.160, 118.63616256], decimal=2
        )
        np.testing.assert_array_almost_equal(
            projs[1]['proj'], [921.506, 458.75711298], decimal=2
        )
        np.testing.assert_array_almost_equal(
            projs[2]['proj'], [830.757, 257.9505475 ], decimal=2
        )
        np.testing.assert_array_almost_equal(
            projs[3]['proj'], [784.80597529, 133.48220928], decimal=2
        )
        np.testing.assert_array_almost_equal(
            projs[4]['proj'], [768.44052177, 396.97917061], decimal=2
        )

    def test_get_occlusions_targets(self):
        """
        Regression test for get_occlusions_targets
        
        The solutions were manually verified visually with the debug outputs
        """
        tgts = copy.deepcopy(self.tgts_visible)
        
        tgt_fake1 = copy.deepcopy(tgts[0])
        tgt_fake1['tvec'][2] += 0.25
        tgts.append(tgt_fake1)
        
        tgt_fake2 = copy.deepcopy(tgts[0])
        tgt_fake2['norm'] *= -1
        tgts.append(tgt_fake2)

        occlusion_results = photogrammetry.get_occlusions_targets(
            self.rmat, self.tvec, tgts, self.vis_checker)

        # Assert all but the last occlusion_results are False
        for occlusion_result in occlusion_results[:-1]:
            self.assertFalse(occlusion_result[0])

        # Assert the last occlusion_result is True and have the following values:
        np.testing.assert_array_almost_equal(
            occlusion_results[-1][1],
            np.array([[-10.70026875], [ -2.03368831], [ -7.]])
        )

    def test_get_visible_targets(self):
        """
        Regression test for get_visible_targets
        
        The solutions were manually verified visually with the debug outputs
        """
        # Add a fake, and not visible target
        tgt_fake = copy.deepcopy(self.tgts_visible[-1])
        tgt_fake['norm'][2] *= -1
        tgts_extra = self.tgts_visible + [tgt_fake]

        # Get the visible targets
        visible_targets = photogrammetry.get_visible_targets(
            self.rmat,
            self.tvec,
            self.cameraMatrix,
            self.distCoeffs,
            tgts_extra,
            self.vis_checker
        )

        # Check that the number of visibile targets is as expected
        self.assertEqual(len(visible_targets), len(tgts_extra) - 1)

    def test_reprojection_error(self):     
        """
        Regression test for reprojection_error
        
        The solutions were manually verified visually with the debug outputs
        """

        # Hardcoded Extrinsics (30160201)
        rmat = np.eye(3)
        tvec = np.array([[0., 0., 0.]])

        # Hardcoded Intrinsics
        cameraMatrix = np.array([[600, 0.0, 512],
                                 [0.0, 660, 256],
                                 [0.0, 0.0, 1.0]])
        distCoeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

        target_tgts = [{'tvec' : [0.,   0., 100.], 'target_type' : 'dot'},
                       {'tvec' : [10.,  0., 100.], 'target_type' : 'dot'},
                       {'tvec' : [0.,  10., 100.], 'target_type' : 'dot'},
                       {'tvec' : [10., 10., 100.], 'target_type' : 'dot'}]
        
        targ_projs = photogrammetry.project_targets(rmat, tvec, cameraMatrix, 
                                                    distCoeffs, target_tgts)

        matching_img_points = []
        for targ_proj in targ_projs:
            matching_img_points.append(
                {'center' : targ_proj['proj'], 'target_type' : 'dot'}
            )
        
        error = photogrammetry.reprojection_error(
            rmat, tvec, cameraMatrix, distCoeffs, target_tgts, matching_img_points)
        
        self.assertEqual(error, (0.0, 0.0))

        matching_img_points[0]['center'][0] += 1
        error = photogrammetry.reprojection_error(
            rmat, tvec, cameraMatrix, distCoeffs, target_tgts, matching_img_points)
        
        self.assertEqual(error, (0.5, 1.0))

        # Note: element 0 still has (+1, +0) since we don't reset it
        matching_img_points[1]['center'][1] += 1
        matching_img_points[3]['center'][1] += 1
        error = photogrammetry.reprojection_error(
            rmat, tvec, cameraMatrix, distCoeffs, target_tgts, matching_img_points)
        
        self.assertEqual(error, (np.sqrt(3)/2, 1.0))


if __name__ == "__main__":
    unittest.main()
