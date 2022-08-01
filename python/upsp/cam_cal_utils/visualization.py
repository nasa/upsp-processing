import matplotlib.pyplot as plt
import numpy as np

debug_text_displays = True


def axisEqual3D(ax):
    """
    In a 3D plot, equalizes all axes to same scale
    """

    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:, 1] - extents[:, 0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize / 2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


def plot_coord_sys(rmat, tvec, ax, scale=10, text=None):
    """
    Given a pose, plots the basis of that coordinate system in the given matplotlib axis

    In the plot, the x axis is blue, the y axis is orange, and the z axis is green

    Inputs:
        rmat - Rotation Matrix. Columns represent the basis of the coordinate system
        tvec - Origin of the coordinate system
        ax - matplotlib axis
        scale - Size of the vcoordinate system vectors
        text - label of the coordinate system
    """

    s_rmat = scale * rmat

    # TODO: these should be drawn in the order of distance to the virtual camera
    #   Drawing them in this order by default can cause the debug outputs to look incorrect
    #   due to occlusions    
    ax.plot3D([tvec[0], tvec[0] + s_rmat[0][0]], [tvec[1], tvec[1] + s_rmat[1][0]], [tvec[2], tvec[2] + s_rmat[2][0]], 'b')
    ax.plot3D([tvec[0], tvec[0] + s_rmat[0][1]], [tvec[1], tvec[1] + s_rmat[1][1]], [tvec[2], tvec[2] + s_rmat[2][1]], 'orange')
    ax.plot3D([tvec[0], tvec[0] + s_rmat[0][2]], [tvec[1], tvec[1] + s_rmat[1][2]], [tvec[2], tvec[2] + s_rmat[2][2]], 'g')

    if (text is not None) and debug_text_displays:
        ax.text(tvec[0], tvec[1], tvec[2], text)


def show_image_locations(img, img_locations, fig_name, scale=5, c='w'):
    """
    Displays an image with the given image locations indicated with a marker of the
        given color (white by default)
    """

    plt.figure(fig_name)
    plt.imshow(img)
    plt.scatter(img_locations[:, 0], img_locations[:, 1], s=scale, c=c)
    plt.savefig(str(fig_name) + '.png')
    plt.close(fig_name)


def show_coord_transforms(cs1_rmat, cs1_tvec, cs2_rmat, cs2_tvec, figname=None,
                          texts=[None, None, None], compares=[False, False, False]):
    """
    Shows the transformation from coordinate system 1 (cs1) to coordinate system 2 (cs2)
        Additionally, shows the origin coordinate system. Draws a line from cs1 to cs2
    Texts is the label to be shown for the origin, cs1, and cs2 respectively
    compares is a list of booleans that determines if a line is drawn from the origin to
        cs1 (compares[0]), from cs1 to cs2 (compares[1]), and from cs2 to the origin
        (compares[2])
    """

    if figname is None:
        figname = 'coordinate system transforms'

    fig = plt.figure(figname)
    ax = fig.gca(projection='3d')

    # Draw the origin, cs1, and cs2
    plot_coord_sys(np.eye(3), (0, 0, 0), ax, text=texts[0])
    plot_coord_sys(cs1_rmat, cs1_tvec, ax, text=texts[1])
    plot_coord_sys(cs2_rmat, cs2_tvec, ax, text=texts[2])

    # If compares[0] is True, draw a line from the origin to cs1
    if compares[0]:
        ax.plot3D([0, cs1_tvec[0]],
                  [0, cs1_tvec[1]],
                  [0, cs1_tvec[2]], 'black')

    # If compare[1] is True, draw a line from cs1 to cs2
    if compares[1]:
        ax.plot3D([cs1_tvec[0], cs2_tvec[0]],
                  [cs1_tvec[1], cs2_tvec[1]],
                  [cs1_tvec[2], cs2_tvec[2]], 'black')

    # If compare[2] is True, draw a line from cs2 to the origin
    if compares[2]:
        ax.plot3D([cs2_tvec[0], 0],
                  [cs2_tvec[1], 0],
                  [cs2_tvec[2], 0], 'black')

    # Equalize and show
    axisEqual3D(ax)
    savefig_name = figname
    if texts[0] is not None:
        savefig_name += '_' + texts[0]
    if texts[1] is not None:
        savefig_name += '_' + texts[1]
    if texts[2] is not None:
        savefig_name += '_' + texts[2]
    
    savefig_name += '.png'
    plt.savefig(savefig_name)
    plt.close(figname)


def plot_pts_and_norms(pts_and_norms, ax, scale=5, c='r'):
    """
    Helper function to plot points with normals

    Input:
        pts_and_norms - list of dictionaries. Each dictionary has keys 'tvec' and 'norm'
            tvec refers to the translation vector from the origin to the point
            norm refers to the point's normal vector
        ax - matplotlib axis
        scale - size of the normal vectors
        c - color of points and normals
    """

    for pt_and_norm in pts_and_norms:
        # Calculate translation vector relative to camera
        tvec = pt_and_norm['tvec']

        # Plot point
        ax.scatter([tvec[0]], [tvec[1]], [tvec[2]], marker='o', c='r', s=scale*2)

        # Calculate scaled normal vector in camera frame
        s_norm = scale * np.array(pt_and_norm['norm'])

        # Plot normal vector
        ax.plot3D([tvec[0], tvec[0] + s_norm[0]],
                  [tvec[1], tvec[1] + s_norm[1]],
                  [tvec[2], tvec[2] + s_norm[2]],
                  c=c)


def show_pts_and_norms(rmat, tvec, pts_and_norms, ax=None, c='r', texts=[None, None]):
    """
    Helper function to plot transformation from the origin to given coordinate system,
        and show a set of points. Originally intended to plot the tgts frame and the
        targets with their normals

    Input:
        rmat - basis of the coordinate system to be shown
        tvec - origin of coordinate system to be shown
        pts_and_norms - list of dictionaries. Each dictionary has keys 'tvec' and 'norm'
            tvec refers to the translation vector from the origin to the point
            norm refers to the point's normal vector. Points and normals are in the
            coordinate frame of the origin
        ax - matplotlib axis
        scale - size of the normal vectors
        c - color of points and normals
        texts - labels for the origin and transformed coordinate system
    """

    # If no axis was given, make a new one
    if ax is None:
        fig = plt.figure('Points & Normals')
        ax = fig.gca(projection='3d')

    # Plot the camera frame, the vector from the camera frame to the tgts frame
    #   and the tgts frame
    plot_coord_sys(np.eye(3), (0, 0, 0), ax, text=texts[0])
    ax.plot3D([0, tvec[0]], [0, tvec[1]], [0, tvec[2]], 'black')
    plot_coord_sys(rmat, tvec, ax, text=texts[1])

    # Plot the targets and their normals
    plot_pts_and_norms(pts_and_norms, ax, c=c)

    axisEqual3D(ax)
    plt.savefig('unfound_visibles.png')
    plt.show()
    plt.close()


def show_projection_matching(img, proj_pts, matching_points, num_matches=None,
                             name='', bonus_pt=None, scale=10., ax=None):
    """
    Show the projected target matching.
        Projected points are labeled in red
        Matching points are labeled in white
        Line connecting point to match is drawn in black

    Input
        img - Display image
        proj_pts - projected location of 3D point
        matching_points - image location of point
        name - figure name prefix
        bonus_pt - optional input. Point to be shown in blue
        scale - scale of point labels
    """

    if num_matches is None:
        num_matches = len(proj_pts)

    plt.figure(name + '_Matched Points')
    plt.imshow(img)
    plt.scatter(proj_pts[:, 0], proj_pts[:, 1], s=scale, c='r')
    plt.scatter(matching_points[:, 0], matching_points[:, 1], s=scale/4, c='w')

    for i, data in enumerate(zip(proj_pts, matching_points)):
        if i >= num_matches:
            break
        proj, match = data
        plt.plot([proj[0], match[0]], [proj[1], match[1]], color='black')

    if bonus_pt is not None:
        plt.scatter([bonus_pt[0]], [bonus_pt[1]], s=scale, c='b')

    plt.savefig(name + '_Matched_Points.png', dpi=400)
    plt.close()