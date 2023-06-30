import unittest
import numpy as np
import os
import cv2
import copy

import file_structure
from upsp.cam_cal_utils import (
    parsers,
    photogrammetry,
    visibility,
)
import upsp.raycast

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


# TODO: This really needs to test the edge cases of the visibility. Viewing angles near
#   the maximum oblique angle, near zero, above it, etc. Just barely visible and not
#   visible. Etc
class VisibilityTestCase(unittest.TestCase):
    """
    Unit tests for the visibility module 
    """

    @classmethod
    def setUpClass(cls):       
        # Checks files set up properly before each running test
        file_structure.setUp(files_dir)

        # Initialize VisibilityChecker
        # Simple scene with several triangle primitives.
        # [t0p0x, t0p0y, t0p0z, t0p1x, t0p1y, t0p1z, t0p2x, t0p2y, t0p2z, ...]
        t0 = [0., 0., 1., 0., 1., 0., 1., 0., 0.]
        t1 = [0., 0., 1., 0., 1., 0., 0., 1., 1.]
        tris = t0 + t1
        
        # Initialize visibility checker with fml_tc3_volume.grid
        cls.vis_checker_nogrid = visibility.VisibilityChecker(
            None, oblique_angle=70, epsilon=1e-4, debug_nogrid=True
        )
        cls.vis_checker_nogrid.scene = upsp.raycast.CreateBVH(tris, 3)
        
        # Initialize VisibilityChecker with fml_tc3_volume.grid
        test_grd = os.path.join(files_dir, 'fml_tc3_volume.grid')
        cls.vis_checker_grid = visibility.VisibilityChecker(
            grid_path=test_grd, oblique_angle=70, epsilon=1e-4
        )
        cls.faces, cls.face_normals = cls.vis_checker_grid.get_faces_and_face_normals()
        cls.nodes, cls.norms = cls.vis_checker_grid.get_tvecs_and_norms()

        # Read in the image
        img_path = os.path.join(
            files_dir, 'images', 'CAM1_RUN8_CINE02_Y20000209H11294501.00001.png'
        )
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE + cv2.IMREAD_ANYDEPTH)

        # Get the camera calibration for this camera
        cam_tun_path = os.path.join(
            files_dir, 'camera-tunnel-calibration', 'camera01_35_6.json'
        )
        
        camera_tunnel_cal = parsers.read_camera_tunnel_cal(cam_tun_path, img.shape)
        cls.rmat, cls.tvec, cls.cameraMatrix, cls.distCoeffs = camera_tunnel_cal

    def test_update_oblique_angle(self):
        """
        Unit test for update_oblique_angle
        """
        self.assertEqual(self.vis_checker_nogrid.oblique_angle, 70)
        self.assertEqual(
            self.vis_checker_nogrid.squared_cos_angle, np.cos(np.deg2rad(70))**2
        )

        self.vis_checker_nogrid.update_oblique_angle(40)
        self.assertEqual(self.vis_checker_nogrid.oblique_angle, 40)
        self.assertEqual(
            self.vis_checker_nogrid.squared_cos_angle, np.cos(np.deg2rad(40))**2
        )

        self.vis_checker_nogrid.update_oblique_angle(70)
        self.assertEqual(self.vis_checker_nogrid.oblique_angle, 70)
        self.assertEqual(
            self.vis_checker_nogrid.squared_cos_angle, np.cos(np.deg2rad(70))**2
        )

    # load_mesh is tested in setUpClass when combined with the regression tests

    # package_primitives is tested in setUpClass when combined with the regression tests

    def test_unit_vector(self):
        """
        Unit test for unit_vector
        """
        vec = np.array([np.sqrt(2), -np.sqrt(2), 0.]).reshape(1, 3)
        np.testing.assert_array_equal(
            self.vis_checker_nogrid.unit_vector(vec), np.array(vec/2)
        )

        vecs = np.array([[np.sqrt(3), -np.sqrt(3), np.sqrt(3)],
                         [np.sqrt(2), -np.sqrt(2),         0.],
                         [0.,                      0.,     1.]]
                       )
        gt = np.divide(vecs, np.array([3, 2, 1]).reshape(3, 1))
        np.testing.assert_array_almost_equal(
            self.vis_checker_nogrid.unit_vector(vecs), gt
        )

        vecs = np.array([[ np.sqrt(3), -np.sqrt(3), np.sqrt(3)],
                         [ np.sqrt(2), -np.sqrt(2),         0.],
                         [-np.sqrt(2), -np.sqrt(2),         0.],
                         [ np.sqrt(2),  np.sqrt(2),         0.],
                         [0.,                      0.,     1.]]
                       )
        gt = np.divide(vecs, np.array([3, 2, 2, 2, 1]).reshape(5, 1))
        np.testing.assert_array_almost_equal(
            self.vis_checker_nogrid.unit_vector(vecs), gt
        )
        np.testing.assert_array_almost_equal(
            self.vis_checker_nogrid.unit_vector(vecs).shape, (5, 3)
        )

    def test_angle_between(self):
        """
        Unit test for angle_between
        """
        x = np.array([[         1.,          0., 0.],
                      [ np.sqrt(2), -np.sqrt(2), 0.],
                      [-np.sqrt(2), -np.sqrt(2), 0.],
                      [         1.,          0., 0.]]
                    )
        y = np.array([[ 0., 1., 0.],
                      [ 0., 1., 0.],
                      [ 0., 1., 0.],
                      [ 0., 1., 0.]]
                    )

        np.testing.assert_array_almost_equal(
            self.vis_checker_nogrid.angle_between(x, y),
            [90., 135., 135., 90.]
        )
        
    # is_back_facing_fast_vectorized is tested indirectly with the following tests

    # does_intersect is tested indirectly with the following tests

    # is_visible is tested indirectly with the following tests

    def test_is_visible_and_inside_incal(self):
        distCoeffs = copy.deepcopy(self.distCoeffs)
        distCoeffs[0][0] *= -1
        distCoeffs[0][1] = -0.4

        R, tvec_model_to_camera = photogrammetry.invTransform(self.rmat, self.tvec)
        visible = self.vis_checker_grid.is_visible(
            tvec_model_to_camera,
            self.nodes,
            self.norms,
        )

        visible_incal = self.vis_checker_grid.is_visible_and_inside_incal(
            self.rmat,
            self.tvec,
            self.cameraMatrix,
            distCoeffs,
            self.nodes,
            self.norms,
            {'critical_pt':'first'}
        )

        self.assertEqual(len(visible_incal), 117553)

    def test_tri_normal(self):
        """
        Unit test for tri_normal
        """
        def unit_vector(vector):
            return (vector / np.linalg.norm(vector)).reshape(3, 1)
        
        face = np.array([[0., 0., 0.], [0., 1., 0.], [0, 0, 1]])
        vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
        np.testing.assert_array_equal(
            unit_vector(vis_check_norm), np.array([1, 0, 0]).reshape(3, 1)
        )

        face = np.array([[0., 0., 0.], [0., 0.5, 0.], [0, 0, 0.5]])
        vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
        np.testing.assert_array_equal(
            unit_vector(vis_check_norm), np.array([1, 0, 0]).reshape(3, 1)
        )

        face = np.array([[0., 0., 0.], [1., 0., 0.], [0, 1, 0]])
        vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
        np.testing.assert_array_equal(
            unit_vector(vis_check_norm), np.array([0, 0, 1]).reshape(3, 1)
        )

        face = np.array([[0., 0., 0.], [0.5, 0., 0.], [0, 0.5, 0]])
        vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
        np.testing.assert_array_equal(
            unit_vector(vis_check_norm), np.array([0, 0, 1]).reshape(3, 1)
        )

        face = np.array([[0., 0., 0.], [1, 0., 0.], [0, 0, 1]])
        vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
        np.testing.assert_array_equal(
            unit_vector(vis_check_norm), np.array([0, -1, 0]).reshape(3, 1)
        )

        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)
        
        # Generate 50 random faces
        random_faces = (2 * rng.random((50, 3, 3))) - 2
        
        # Test photogrammetry.rot against the cached version
        for face in random_faces:
            A = face[1] - face[0]
            B = face[2] - face[1]
            vis_check_norm = self.vis_checker_nogrid.tri_normal(face)
            np.testing.assert_array_equal(
                unit_vector(vis_check_norm), unit_vector(np.cross(A, B))
            )

    def test_get_faces_and_face_normals(self):
        """
        Unit test for get_faces_and_face_normals
        """
        faces, face_normals = self.vis_checker_grid.get_faces_and_face_normals()
        self.assertEqual(faces.shape, (609120, 3, 3))
        self.assertEqual(face_normals.shape, (609120, 3))

    def test_get_tvecs_and_norms(self):
        """
        Unit test for get_tvecs_and_norms
        """
        tvecs, norms = self.vis_checker_grid.get_tvecs_and_norms()
        self.assertEqual(tvecs.shape, (304566, 3))
        self.assertEqual(norms.shape, (304566, 3))

    def test_camera01(self):
        """
        Regression test of is_visible and indirectly tests load_mesh,
        package_primitives, is_back_facing (or similar), and does_intersect
        """
        R, tvec_model_to_camera = photogrammetry.invTransform(self.rmat, self.tvec)
        visible = self.vis_checker_grid.is_visible(
            tvec_model_to_camera,
            self.nodes,
            self.norms,
        )
        self.assertEqual(len(visible), 148608)

    def test_back_facing(self):
        """
        Regression test of is_visible and indirectly tests load_mesh,
        package_primitives, is_back_facing (or similar), and does_intersect
        """
        camera_t = np.array([1., 1., 1.]).reshape(3, 1)
        nodes = np.array([[2., 2., 2.], [3., 3., 3.]]).reshape(-1, 3)
        normals = np.array([[1., 1., 1.], [1., 1., 1.]]).reshape(-1, 3)

        result = self.vis_checker_nogrid.is_visible(camera_t, nodes, normals)
        self.assertEqual(result.tolist(), [])

    def test_occluded(self):
        """
        Regression test of is_visible and indirectly tests load_mesh,
        package_primitives, is_back_facing (or similar), and does_intersect
        """
        camera_t = np.array([1., 1., 1.]).reshape(3, 1)
        nodes = np.array([[0., 0., 0.], [-1., -1., -1.]]).reshape(-1, 3)
        normals = np.array([[0.9, 0.9, 0.9], [1.1, 1.1, 1.1]]).reshape(-1, 3)

        result = self.vis_checker_nogrid.is_visible(camera_t, nodes, normals)
        self.assertEqual(result.tolist(), [])

    def test_all_visible(self):
        """
        Regression test of is_visible and indirectly tests load_mesh,
        package_primitives, is_back_facing (or similar), and does_intersect
        """
        camera_t = np.array([-8., -8., 0.])
        nodes = np.array([[-5., -5., 0.], [-5., -1., 0]]).reshape(-1, 3)
        normals = np.array([[-0.9, 0., 0.], [0., -1.2, 0.]]).reshape(-1, 3)

        result = self.vis_checker_nogrid.is_visible(camera_t, nodes, normals)
        self.assertEqual(result.tolist(), [0, 1])

        camera_t = np.array([-2., -2., -2.]).reshape(3, 1)
        nodes = np.array([[-1., -1., -1.], [0., 0., 0.]]).reshape(-1, 3)
        normals = np.array([[-1., -1., -1.], [-1., -1., -1.]]).reshape(-1, 3)

        result = self.vis_checker_nogrid.is_visible(camera_t, nodes, normals)
        self.assertEqual(result.tolist(), [0, 1])

    def test_some_visible(self):
        """
        Regression test of is_visible and indirectly tests load_mesh,
        package_primitives, is_back_facing (or similar), and does_intersect
        """
        camera_t = np.array([1., 1., 1.]).reshape(3, 1)
        nodes = [[2., 2., 2.],
                 [3., 3., 3.],
                 [0.9, 0.9, 0.9],
                 [0.5, 0.5, 0.5],
                 [0., 0., 0.],
                 [-1., -1., -1.]]
        nodes = np.array(nodes).reshape(-1, 3)
        
        normals = [[1., 1., 1.],
                   [1., 1., 1.],
                   [1., 1., 1.],
                   [1., 1., 1.],
                   [1., 1., 1.],
                   [1., 1., 1.]]
        normals = np.array(normals).reshape(-1, 3)
        
        result = self.vis_checker_nogrid.is_visible(camera_t, nodes, normals)
        self.assertEqual(result.tolist(), [2, 3])


if __name__ == "__main__":
    unittest.main()
