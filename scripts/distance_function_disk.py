import sys
import numpy as np
from cspace_visualizer import *
#import pandas as pa
import matplotlib.pyplot as plt
#import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.tri as mtri
from matplotlib.colors import Normalize, colorConverter, LightSource
from mpl_toolkits.mplot3d import art3d

#def plot_trisurf_mask(ax, *args, **kwargs):
#    """
#    ============= ================================================
#    Argument      Description
#    ============= ================================================
#    *X*, *Y*, *Z* Data values as 1D arrays
#    *color*       Color of the surface patches
#    *cmap*        A colormap for the surface patches.
#    *norm*        An instance of Normalize to map values to colors
#    *vmin*        Minimum value to map
#    *vmax*        Maximum value to map
#    *shade*       Whether to shade the facecolors
#    ============= ================================================
#    The (optional) triangulation can be specified in one of two ways;
#    either::
#      plot_trisurf(triangulation, ...)
#    where triangulation is a :class:`~matplotlib.tri.Triangulation`
#    object, or::
#      plot_trisurf(X, Y, ...)
#      plot_trisurf(X, Y, triangles, ...)
#      plot_trisurf(X, Y, triangles=triangles, ...)
#    in which case a Triangulation object will be created.  See
#    :class:`~matplotlib.tri.Triangulation` for a explanation of
#    these possibilities.
#    The remaining arguments are::
#      plot_trisurf(..., Z)
#    where *Z* is the array of values to contour, one per point
#    in the triangulation.
#    Other arguments are passed on to
#    :class:`~mpl_toolkits.mplot3d.art3d.Poly3DCollection`
#    **Examples:**
#    .. plot:: mpl_examples/mplot3d/trisurf3d_demo.py
#    .. plot:: mpl_examples/mplot3d/trisurf3d_demo2.py
#    .. versionadded:: 1.2.0
#        This plotting function was added for the v1.2.0 release.
#    """

#    had_data = ax.has_data()

#    # TODO: Support custom face colours
#    color = np.array(colorConverter.to_rgba(kwargs.pop('color', 'b')))

#    cmap = kwargs.get('cmap', None)
#    norm = kwargs.pop('norm', None)
#    vmin = kwargs.pop('vmin', None)
#    vmax = kwargs.pop('vmax', None)
#    linewidth = kwargs.get('linewidth', None)
#    shade = kwargs.pop('shade', cmap is None)
#    lightsource = kwargs.pop('lightsource', None)

#    tri, args, kwargs = mtri.Triangulation.get_from_args_and_kwargs(*args, **kwargs)

#    # mask = mtri.TriAnalyzer(tri).get_flat_tri_mask()
#    # tri.set_mask(mask)
#    #def long_edges(x, y, triangles, radio=1):
#    #    out = []
#    #    for points in triangles:
#    #        a,b,c = points
#    #        d0 = np.sqrt( (x[a] - x[b]) **2 + (y[a] - y[b])**2 )
#    #        d1 = np.sqrt( (x[b] - x[c]) **2 + (y[b] - y[c])**2 )
#    #        d2 = np.sqrt( (x[c] - x[a]) **2 + (y[c] - y[a])**2 )
#    #        max_edge = max([d0, d1, d2])
#    #        #print points, max_edge
#    #        if max_edge > radio:
#    #            out.append(True)
#    #        else:
#    #            out.append(False)
#    #    return out

#    #mask = long_edges(tri.x, tri.y, tri.triangles)
#    #tri.set_mask(mask)

#    z = np.asarray(args[0])

#    triangles = tri.get_masked_triangles()
#    xt = tri.x[triangles][...,np.newaxis]
#    yt = tri.y[triangles][...,np.newaxis]
#    zt = np.array(z)[triangles][...,np.newaxis]

#    verts = np.concatenate((xt, yt, zt), axis=2)

#    # Only need these vectors to shade if there is no cmap
#    if cmap is None and shade:
#        totpts = len(verts)
#        v1 = np.empty((totpts, 3))
#        v2 = np.empty((totpts, 3))
#        # This indexes the vertex points
#        which_pt = 0

#    colset = []
#    for i in xrange(len(verts)):
#        avgzsum = verts[i,0,2] + verts[i,1,2] + verts[i,2,2]
#        colset.append(avgzsum / 3.0)

#        # Only need vectors to shade if no cmap
#        if cmap is None and shade:
#            v1[which_pt] = np.array(verts[i,0]) - np.array(verts[i,1])
#            v2[which_pt] = np.array(verts[i,1]) - np.array(verts[i,2])
#            which_pt += 1

#    if cmap is None and shade:
#        normals = np.cross(v1, v2)
#    else:
#        normals = []

#    polyc = art3d.Poly3DCollection(verts, *args, **kwargs)

#    if cmap:
#        colset = np.array(colset)
#        polyc.set_array(colset)
#        if vmin is not None or vmax is not None:
#            polyc.set_clim(vmin, vmax)
#        if norm is not None:
#            polyc.set_norm(norm)
#    else:
#        if shade:
#            colset = ax._shade_colors(color, normals)
#        else:
#            colset = color
#        polyc.set_facecolors(colset)

#    ax.add_collection(polyc)
#    ax.auto_scale_xyz(tri.x, tri.y, z, had_data)

#    return polyc

font_size = 45
showSamples = True

folder_name = "../data/samples/"
fname = folder_name+"02D_disk_narrow_passage.samples"

Q = getPoints(fname)
P1 = []
P2 = []
P3 = []
dmax_index = -1
dmax = 0
ctr = 0
for q in Q:
  feasible = q[0]
  sufficient = q[1]
  ball_radius = q[2]
  number_of_states = q[3]
  x = q[4]
  y = q[5]
  P1 = np.append(P1,x)
  P2 = np.append(P2,y)
  P3 = np.append(P3,ball_radius)
  if ball_radius > dmax:
    dmax = ball_radius
    dmax_index = ctr
  ctr = ctr+1

P1 = np.array(P1,dtype='float')
P2 = np.array(P2,dtype='float')
P3 = np.array(P3,dtype='float')

# P1 = np.delete(P1, dmax_index, axis=0)
# P2 = np.delete(P2, dmax_index, axis=0)
# P3 = np.delete(P3, dmax_index, axis=0)

print P1.shape, P2.shape, P3.shape

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(P1,P2,P3,"r")

ax.set_xlabel('X [m]', labelpad=20)
ax.set_ylabel('Y [m]', labelpad=20)
ax.set_zlabel('d [m]', rotation=90, labelpad=20)
ax.dist = 10


# X,Y = np.meshgrid(P1, P2)
# Z = griddata((df['x'], df['y']), df['z'], (P1, P2), method='cubic')

# surf = ax.plot_surface(P1, P2, P3, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)
# 3
tri = mtri.Triangulation(P1,P2)

# mask = mtri.TriAnalyzer(tri).get_flat_tri_mask()
# tri.set_mask(mask)

#def long_edges(x, gpy, triangles, radio=0.001):
#    out = []
#    for points in triangles:
#        a,b,c = points
#        d0 = np.sqrt( (x[a] - x[b]) **2 + (y[a] - y[b])**2 )
#        d1 = np.sqrt( (x[b] - x[c]) **2 + (y[b] - y[c])**2 )
#        d2 = np.sqrt( (x[c] - x[a]) **2 + (y[c] - y[a])**2 )
#        max_edge = max([d0, d1, d2])
#        #print points, max_edge
#        print points, max_edge
#        if max_edge > radio:
#            out.append(True)
#        else:
#            out.append(False)
#    return out

#mask = long_edges(P1, P2, tri.triangles)
#tri.set_mask(mask)

# mask = np.all(np.where(isbad[tri.triangles], True, False), axis=1)
# triang.set_mask(mask)

#surf = ax.plot_trisurf(P1, P2, P3, cmap=cm.jet, linewidth=0.5, edgecolor='none', antialiased=True)
surf = ax.plot_trisurf(P1, P2, P3, triangles=tri.triangles, linewidth=0,
    cmap=plt.cm.RdGy, antialiased=False, edgecolor='none')

plt.show()
