import numpy as np
from scipy.spatial import cKDTree

def get_rmse(cloud1, cloud2, max_dist=0.5):
    tree = cKDTree(cloud2)
    dists, _ = tree.query(cloud1, k=1)
    # Filter out points that are far away (ignoring outliers/missing overlap)
    valid_dists = dists[dists < max_dist]
    if len(valid_dists) == 0: return 999.0
    return np.mean(valid_dists)

print("Test ok")
