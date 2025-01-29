"""
Offline Localization using Belief Propagation

This class estimates unknown positions on a grid using belief propagation, 
which updates probabilities based on distance measurements from known points. 
It is useful for localization in sensor networks, robotics, and autonomous 
vehicle tracking.

Key Features:
- Uses a grid-based belief system to refine position estimates.
- Updates beliefs with Gaussian probability based on measured distances.
- Identifies likely positions by selecting the highest belief value.

This approach is particularly effective in noisy environments where 
measurements are uncertain, and adjusting the grid resolution and noise 
parameters can improve accuracy.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

class OfflineLocalization:
    def __init__(self, grid_size=1, area_size=100, noise_std=0.5):
        self.grid_size = grid_size  # Resolution of belief grid
        self.area_size = area_size
        self.noise_std = noise_std  # Standard deviation of measurement noise

    def initialize_belief_grid(self):
        """Creates a grid for belief propagation."""
        grid_x, grid_y = np.meshgrid(
            np.arange(0, self.area_size, self.grid_size),
            np.arange(0, self.area_size, self.grid_size)
        )
        return grid_x, grid_y, np.ones(grid_x.shape)  # Initialize belief

    def belief_propagation(self, known_positions, distances):
        """Uses belief propagation to estimate vehicle locations on a grid."""
        unknown_nodes = set(node for edge in distances for node in edge) - set(known_positions.keys())
        estimated_positions = {}

        # Iterate over each unknown node
        for node in unknown_nodes:
            grid_x, grid_y, belief = self.initialize_belief_grid()

            # Iterate over known neighbors
            for (nbr, nbr_pos, d) in [(nbr, known_positions[nbr], distances[(node, nbr)])
                                       for (a, b) in distances if node in (a, b)
                                       for nbr in ([a] if node == b else [b])
                                       if nbr in known_positions]:
                dist_map = np.linalg.norm(np.column_stack((grid_x.ravel(), grid_y.ravel())) - nbr_pos, axis=1)
                dist_map = dist_map.reshape(grid_x.shape)

                # Update belief using Gaussian probability
                belief *= norm.pdf(dist_map, loc=d, scale=self.noise_std)

            # Normalize belief
            belief /= np.max(belief)

            # Find estimated position (grid point with max belief)
            max_idx = np.unravel_index(np.argmax(belief), belief.shape)
            estimated_positions[node] = np.array([grid_x[max_idx], grid_y[max_idx]])

            print(f"📍 Estimated position for node {node}: {estimated_positions[node]}")

        return estimated_positions

    def locate_other_vehicles(self, known_positions, distances):
        """Wrapper function to ensure compatibility with main.py"""
        return self.belief_propagation(known_positions, distances)
