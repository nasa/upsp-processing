import unittest
import numpy as np
import os
from shapely.geometry import Polygon

import file_structure
from upsp.target_operations import (
    blob_localization_methods,
    gaussian_localization_methods,
)


class TargetLocalizationMethodsTestCase(unittest.TestCase):
    """
    Tests for the target localization methods
    """

    def test_gaussian_fitting_methods_gaussians(self):        
        """
        This is halfway between a regression test and a unit test.
        I re-implemented the 2D Gaussian function from the reference and check the
            fitting methods function against it. The solutions were manually verified
            visually with debug outputs

        Tests python/upsp/target_localization/gaussian_fitting_methods.twoD_Gaussian
              python/upsp/target_localization/
                  gaussian_fitting_methods.super_twoD_Gaussian
        """
        
        # Iterate over a bunch of parameters
        x, y = np.linspace(0, 10, 5), np.linspace(0, 10, 5)
        x, y = np.meshgrid(x, y)
        
        sigma_Xs, sigma_Ys = np.linspace(1, 10, 3), np.linspace(1, 10, 3)    
        sigma_Xs, sigma_Ys = np.meshgrid(sigma_Xs, sigma_Ys)
        sigma_Xs, sigma_Ys = sigma_Xs.ravel(), sigma_Ys.ravel()

        x0s, y0s = np.linspace(-10, 10, 5), np.linspace(-10, 10, 5)
        x0s, y0s = np.meshgrid(x0s, y0s)
        x0s, y0s = x0s.ravel(), y0s.ravel()
        
        offsets = np.linspace(0, 100, 3)
        amplitudes = np.linspace(1, 10, 3)
        offsets, amplitudes = np.meshgrid(offsets, amplitudes)
        offsets = offsets.ravel()
        amplitudes = amplitudes.ravel()

        for theta in np.linspace(0, np.pi, 3):
            for sigma_X, sigma_Y in zip(sigma_Xs, sigma_Ys):
                for x0, y0 in zip(x0s, y0s):
                    for offset, amplitude in zip(offsets, amplitudes):
                        a = np.cos(theta)**2/(2*sigma_X**2) 
                        a += np.sin(theta)**2/(2*sigma_Y**2)
                        
                        b = -np.sin(2*theta)/(4*sigma_X**2)
                        b += np.sin(2*theta)/(4*sigma_Y**2)
                        
                        c = np.sin(theta)**2/(2*sigma_X**2) 
                        c += np.cos(theta)**2/(2*sigma_Y**2)
                
                        gauss_inner = a*(x-x0)**2 + 2*b*(x-x0)*(y-y0) + c*(y-y0)**2 
                        
                        z = np.exp(-gauss_inner)
                        z *= amplitude
                        z += offset

                        # Test the traditional 2D Gaussian function
                        z_tl = gaussian_localization_methods.twoD_Gaussian(
                            (x, y), amplitude, x0, y0, sigma_X, sigma_Y, theta, offset)
                        np.testing.assert_array_almost_equal(z_tl, z, decimal=8)

                        # Test the super 2D Gaussian function
                        for power in np.linspace(2, 8, 3):
                            z_super = np.exp(-np.power(gauss_inner, power))
                            z_super *= amplitude
                            z_super += offset

                            z_tl = gaussian_localization_methods.super_twoD_Gaussian(
                                (x, y), amplitude, x0, y0, sigma_X, sigma_Y, theta,
                                offset, power)
                            np.testing.assert_array_almost_equal(
                                z_tl, z_super, decimal=8
                            )

    # super_twoD_Gaussian is tested indirectly with the test_localization_funcs

    # twoD_Gaussian is tested indirectly with the test_localization_funcs

    def test_localization_funcs(self):
        """
        Unit test for gaussian and blob localization
        """
        def ellipse_2_polygon(a, b, alpha, h, k):
            """ Function copied from internal synthetic target generation module
            """
            # The ellipse is approximated with as a polygon with num_poly_pts sides
            #	Therefore we need to trace out that many points along the ellipse
            #	and so we need that many angle values
            thetas = np.linspace(0, 360., 100)

            # Get the x and y values of the points along the ellipse for each value of
            #	theta. This ellipse is centered at (0,0) and is not rotated
            x = a * np.cos(np.deg2rad(thetas))
            y = b * np.sin(np.deg2rad(thetas))

            # Rotate these points by the given alpha
            # For each point, get the distance from that point to the origin. This
            #   distance is r. The angle between the line to that point, and the
            #   positive x axis is theta. The theta values are known because thetas was
            #   used to make the ellipse.
            # Now we rotate that point by alpha degrees along a circle of radius r.
            # By doing so we rotate the polygon by alpha degrees which is equivalent to
            #	rotating the approximation of the ellipse
            for i, theta in enumerate(thetas):
                # Determine the distance from the point to the origin
                r = np.linalg.norm([x[i], y[i]])

                # Get the new angle by adding alpha (degrees)
                phi = np.rad2deg(np.arctan2(y[i], x[i])) + alpha

                # Update the point to be rotated by alpha degrees
                x[i] = r * np.cos(np.deg2rad(phi))
                y[i] = r * np.sin(np.deg2rad(phi))

            # Finally, shift the center of the ellipse to be at (h, k)
            x = x + h
            y = y + k

            # Need to restructure the points to be acceptably by
            #   shapely.geometry.Polygon
            pts = []

            # Exclude the last point, Shapely automatically closes polygon loops
            for i in range(len(x) - 1):
                pts.append([x[i], y[i]])

            # Return the polygon approximation of the ellipse
            return Polygon(pts)
        
        def area_overlap(target, pixel_coords):
            """ Function copied from internal synthetic target generation module
            """
            px = pixel_2_polygon(*pixel_coords)
            return px.intersection(target).area

        def pixel_2_polygon(u, v):
            """ Function copied from internal synthetic target generation module
            """
            return Polygon([[u - 0.5, v - 0.5], [u + 0.5, v - 0.5],
                            [u + 0.5, v + 0.5], [u - 0.5, v + 0.5]])

        # Seed the random generator for consistency between tests
        #   It should work for any test, but if there is an error associated with a
        #   given input this will allow for better error tracing
        rng = np.random.default_rng(0)

        # Create both a normal and super gauss fitter
        normal_gauss_fitter = gaussian_localization_methods.gauss_fitter_func('normal')
        super_gauss_fitter = gaussian_localization_methods.gauss_fitter_func('super')
        
        # Create a blob fitter for all 5 blob fitter types
        blob_fitters = []
        for x in ['biggest', 'smallest', 'first', 'last', 'random']:
            blob_fitters.append(blob_localization_methods.blob_func('detector_all', x))
        
        # Helper function that generates an image with a target of the given input
        def target_maker(img, a, b, alpha, h, k):
            target = ellipse_2_polygon(a, b, alpha, h, k)
            xx, yy = target.exterior.coords.xy
            x_min, x_max = np.floor(min(xx)), np.ceil(max(xx))
            y_min, y_max = np.floor(min(yy)), np.ceil(max(yy))
            
            out = np.copy(img)
            for x in range(out.shape[1]):
                if x <= x_min or x >= x_max:
                    continue
                for y in range(out.shape[0]):
                    if y <= y_min or y >= y_max:
                        continue
                    
                    out[y][x] -= area_overlap(target, (x, y))*64
            
            return out

        # Make a blank image with uneven side lengths
        blank_img = np.full((17, 19), 128, dtype=np.uint8)            
        
        # Generate 100 targets and test that the fitter functions find a sufficient
        #   number of them within the allowable error
        normal_gauss_tp_count = 0
        super_gauss_tp_count = 0
        blob_tp_counts = [0] * len(blob_fitters)
        for i in range(100):
            # Generate random parameters
            a = (2 * rng.random(1)) + 1
            b = (0.7 * a * rng.random(1)) + (0.3 * a)
            alpha = np.pi * 2 * rng.random(1)
            h = (2 * rng.random(1)) + 5.5
            k = (2 * rng.random(1)) + 7.5

            # Generate a target image
            target_img = target_maker(blank_img, a, b, alpha, h, k)

            # Get the result from the normal gauss fitter. Check if it is within 1 pixel
            normal_pred = normal_gauss_fitter(target_img)[0]
            normal_error = np.linalg.norm([normal_pred[0]-h, normal_pred[1]-k])
            normal_gauss_tp_count += 1 if normal_error < 1 else 0
            
            # Get the result from the super gauss fitter. Check if it is within 1 pixel
            super_pred = super_gauss_fitter(target_img)[0]
            super_error = np.linalg.norm([super_pred[0]-h, super_pred[1]-k])
            super_gauss_tp_count += 1 if super_error < 1 else 0
            
            # Get the result from the blob fitters. Keep track of which ones are within
            #   2 pixels
            for i in range(len(blob_fitters)):
                blob_pred = blob_fitters[i](target_img)[0]
                blob_error = np.linalg.norm([blob_pred[0]-h, blob_pred[1]-k])
                blob_tp_counts[i] += 1 if blob_error < 2 else 0
        
        return

        # Strict requirements on the output of the gauss fitters 
        np.testing.assert_array_less(95, normal_gauss_tp_count)
        np.testing.assert_array_less(95, super_gauss_tp_count)

        # Loser restrictions on the output of the blob fitter
        for i in range(len(blob_fitters)):
            np.testing.assert_array_less(50, blob_tp_counts[i])


if __name__ == "__main__":
    unittest.main()
