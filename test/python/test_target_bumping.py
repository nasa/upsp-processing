import unittest
import numpy as np
import os
import copy

import file_structure
from upsp.target_operations import target_bumping
from upsp.cam_cal_utils import (
    parsers,
    visibility,
)

files_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data')


class TargetBumpingTestCase(unittest.TestCase):
    """
    Tests for target_bumping
    """

    @classmethod
    def setUpClass(cls):       
        # Read the rest configuration file
        test_config_file = os.path.join(files_dir, 'test_config.json')
        cls.test_config = parsers.read_json(test_config_file)

        # Load the tgts file
        cls.tgts_file_path = os.path.join(files_dir, 'fml_tc3_volume.tgts')
        cls.tgts = parsers.read_tgts(cls.tgts_file_path)

        # Get the visibility checker from the grid
        grd_file = os.path.join(files_dir, 'fml_tc3_volume.grid')
        
        cls.vis_checker = visibility.VisibilityChecker(
            grd_file, 70, epsilon=1e-4
        )

    def setUp(self):
        """
        Checks files set up properly before each running test
        """

        target_bumping.debug_print_bumping = False
        target_bumping.debug_bump_distance = False

        file_structure.setUp(files_dir)

    def tearDown(self):
        """
        Checks files are still set up properly after each running test
        """

        # We can rerun setUp because the file structure should be unchanged
        self.setUp()

    # target_bumping.get_bumping_occlusion is tested via tgt_bump_internals and
    #   tgt_bump_externals

    # target_bumping.is_real_occlusion is tested via tgt_bump_internals

    def test_tgts_get_internals(self):
        """
        Regression test for tgts_get_internals

        The solutions were manually verified visually with the debug outputs
        """
        tgts = copy.deepcopy(self.tgts)
        internals = [1, 2, 3, 5, 8, 13, 21]
        internal_names_gt = [tgts[i]['name'] for i in internals]
        for i, tgt in enumerate(tgts):
            if i in internals:
                tgt['tvec'] -= 0.02 * tgt['norm']

        # Get the bumped targets
        internals = target_bumping.tgts_get_internals(
            tgts, self.vis_checker, tgts_tol=1e-4, grid_tol=1e-3
        )
        
        internal_names = []
        for internal in internals:
            if internal[2]:
                internal_names.append(internal[0])
        
        self.assertEqual(internal_names, internal_names_gt)

    def test_tgt_bump_internals(self):    
        """
        Regression test for tgt_bump_internals

        The solutions were manually verified visually with the debug outputs
        """
        tgts = copy.deepcopy(self.tgts)
        internals = [1, 2, 3, 5, 8, 13, 21]
        internal_names_gt = [tgts[i]['name'] for i in internals]
        for i, tgt in enumerate(tgts):
            if i in internals:
                tgt['tvec'] -= 0.02 * tgt['norm']
        
        # Get the bumped targets
        bumped_internals, was_bumped = target_bumping.tgt_bump_internals(
            tgts, self.vis_checker, bump_eps=1e-5, tgts_tol=1e-1, grid_tol=1e-3
        )

        for i, tgt in enumerate(tgts):
            for bi in bumped_internals:
                if tgt['name'] == bi['name']:
                    diff = tgt['tvec'] - bi['tvec']
                    if i in internals:
                        self.assertAlmostEqual(np.linalg.norm(diff), 0.02, 2)
                    else:
                        self.assertLess(np.linalg.norm(diff), 1e-3)

        # Check that the bumped targets don't change when bumping them again
        #   Since they should be outside the model it shouldn't modify them
        # We know from manually checking that double bumping shouldn't change them
        #   so long as the tgt_bump_internals function is working properly
        double_bumped_internals, was_bumped = target_bumping.tgt_bump_internals(
            bumped_internals,
            self.vis_checker,
            bump_eps=1e-5,
            tgts_tol=1e-4,
            grid_tol=1e-3
        )
        
        # Targets should not be changed in the double bumped, check them against the
        #   single bumped ones
        for double, single in zip(double_bumped_internals, bumped_internals):
            np.testing.assert_array_almost_equal(
                double['tvec'], single['tvec'], decimal=3
            )
            np.testing.assert_array_almost_equal(
                double['norm'], single['norm'], decimal=3
            )

    def test_tgt_bump_externals(self):
        """
        Regression test for tgt_bump_externals

        The solutions were manually verified visually with the debug outputs
        """
        tgts = copy.deepcopy(self.tgts)
        internals = [1, 2, 3, 5, 8, 13, 21]
        internal_names_gt = [tgts[i]['name'] for i in internals]
        for i, tgt in enumerate(tgts):
            if i in internals:
                tgt['tvec'] += 0.02 * tgt['norm']
        
        # Do the external bumping
        externals = target_bumping.tgt_bump_externals(
            tgts, self.vis_checker, bump_eps=1e-5
        )

        for i, tgt in enumerate(tgts):
            for ex in externals:
                if tgt['name'] == ex['name']:
                    diff = tgt['tvec'] - ex['tvec']
                    if i in internals:
                        self.assertAlmostEqual(np.linalg.norm(diff), 0.02, 2)
                    else:
                        self.assertLess(np.linalg.norm(diff), 1e-3)
    
    # target_bumping.tgts_bumper is tested via tgts_file_bumper

    def test_tgts_file_bumper(self):
        """
        Regression test for tgts_file_bumper

        The solutions were manually verified visually with the debug outputs
        """
        new_tgts_file_path = os.path.join(files_dir, 'output', 'tgts_bumped.tgts')
            
        target_bumping.tgts_file_bumper(
            self.tgts_file_path,
            self.vis_checker,
            bump_eps=1e-1,
            tgts_tol=1e-1,
            grid_tol=1e-3,
            new_tgts_file_path=new_tgts_file_path
        )
        
        bumped_tgts = parsers.read_tgts(new_tgts_file_path)

        for tgt, bumped_tgt in zip(self.tgts, bumped_tgts):
            diff = tgt['tvec'] - bumped_tgt['tvec']
            self.assertAlmostEqual(np.linalg.norm(diff), 0.1, 2)
        
        # Deleted the generated bumped tgts file
        os.remove(new_tgts_file_path)
        
if __name__ == "__main__":
    unittest.main()
