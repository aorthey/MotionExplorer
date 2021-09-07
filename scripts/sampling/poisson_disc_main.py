import numpy as np
import poisson_disc as pd
# plotting tools
# %matplotlib widget
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':

    # default: 2D points, classic Bridson algorithm as described here:
    # https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
    points = pd.Bridson_sampling()

    fig, ax = plt.subplots(1, 2, figsize=(10,20))
    ax[0].scatter(points[:,0], points[:,1], s=10)
    ax[0].set_xlim(0, 1)
    ax[0].set_ylim(0, 1)
    ax[0].set_aspect('equal')
    
    # alternative sampler, results in denser points: 
    # based on the method proposed here by Martin Roberts: http://extremelearning.com.au/an-improved-version-of-bridsons-algorithm-n-for-poisson-disc-sampling/
    dims2d = np.array([1.0,1.0])
    points_surf = pd.Bridson_sampling(dims=dims2d, radius=0.05, k=30, hypersphere_sample=pd.hypersphere_surface_sample)
    
    ax[1].scatter(points_surf[:,0], points_surf[:,1], s=10)
    ax[1].set_xlim(0, 1)
    ax[1].set_ylim(0, 1)
    ax[1].set_aspect('equal')
    
    # 3D points
    # dims3d = np.array([1.0,1.0,1.0])
    # points3d = pd.Bridson_sampling(dims3d, radius=0.05, k=30)
    # fig3d = plt.figure()
    # ax3d = Axes3D(fig3d)
    # ax3d.scatter(points3d[:,0], points3d[:,1], points3d[:,2])
    plt.show()

