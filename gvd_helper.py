
import numpy as np
from skimage.morphology import medial_axis
from scipy.ndimage import convolve, label, center_of_mass

def build_gvd_from_grid(grid_map):
    """
    grid_map: np.ndarray, 0=free, 1=obstacle, 2=goal(optional, treated as free)

    Returns a medial-axis approximation of the Voronoi Boundary (VB)
    suitable for later graph construction.
    """
    work = np.array(grid_map, copy=True)
    work[work == 2] = 0

    free_mask = (work == 0)
    gvd_mask, distance = medial_axis(free_mask, return_distance=True)

    neighbor_count = convolve(
        gvd_mask.astype(np.uint8),
        np.ones((3, 3), dtype=np.uint8),
        mode="constant",
        cval=0
    ) - gvd_mask.astype(np.uint8)

    # Raw skeleton junction pixels
    branch_mask = gvd_mask & (neighbor_count >= 3)
    end_mask = gvd_mask & (neighbor_count == 1)

    # Cluster adjacent branch pixels into one Voronoi node each
    branch_labels, num_branch_components = label(
        branch_mask, structure=np.ones((3, 3), dtype=np.uint8)
    )
    branch_centroids = center_of_mass(
        branch_mask.astype(np.uint8),
        branch_labels,
        range(1, num_branch_components + 1)
    )
    branch_centroids = np.array(
        [(int(round(r)), int(round(c))) for r, c in branch_centroids],
        dtype=np.int32
    )

    return {
        "free_mask": free_mask,
        "gvd_mask": gvd_mask,
        "distance": distance,
        "branch_mask_raw": branch_mask,
        "end_mask": end_mask,
        "gvd_points": np.argwhere(gvd_mask),
        "branch_pixels_raw": np.argwhere(branch_mask),
        "branch_centroids": branch_centroids,
        "end_points": np.argwhere(end_mask),
        "neighbor_count": neighbor_count,
    }


def plot_gvd(grid_map, gvd_result, ax=None):
    import matplotlib.pyplot as plt

    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    ax.imshow(grid_map, cmap="gray_r", origin="upper", interpolation="nearest")

    gvd_points = gvd_result["gvd_points"]
    branch_centroids = gvd_result["branch_centroids"]

    if len(gvd_points) > 0:
        ax.scatter(gvd_points[:, 1], gvd_points[:, 0], s=10, c="deepskyblue", label="GVD")
    if len(branch_centroids) > 0:
        ax.scatter(branch_centroids[:, 1], branch_centroids[:, 0], s=40, c="red", marker="x", label="Voronoi nodes")

    ax.set_title("Generalized Voronoi Diagram (medial-axis approximation)")
    ax.set_aspect("equal")
    ax.legend()
    return ax
