"""
Network Simulator for Vehicle Communication

This class simulates a network of vehicles within a defined area, establishing 
connections based on a given communication range. It randomly places vehicles, 
creates network edges based on distance constraints, and ensures full connectivity.

Key Features:
- Generates a random network of vehicles and anchor nodes.
- Establishes edges between vehicles based on communication range.
- Ensures each vehicle has at least three neighbors for stability.
- Provides visualization of the network topology.

Note: Please avoid modifying the structure unless necessary, as changes may 
affect connectivity enforcement and visualization accuracy.
"""

import matplotlib.pyplot as plt
import numpy as np
import random

class NetworkSimulator:
    def __init__(self, num_vehicles=10, comm_range=70, area_size=100, num_anchors=3):
            self.num_vehicles = num_vehicles
            self.comm_range = comm_range
            self.area_size = area_size
            self.num_anchors = num_anchors
            self.vehicle_positions = self._generate_vehicle_positions()
            self.edges = self._generate_edges()
            self.anchors = self._select_anchors()
            self.ensure_full_connectivity()


    def _generate_vehicle_positions(self):
        """Generate random positions for vehicles within the simulation area."""
        positions = {i: (random.uniform(0, self.area_size), random.uniform(0, self.area_size)) for i in range(self.num_vehicles)}
        return positions

    def _generate_edges(self):
        """Generate edges based on communication range."""
        edges = {}
        for i in range(self.num_vehicles):
            for j in range(i + 1, self.num_vehicles):
                d = self._distance(self.vehicle_positions[i], self.vehicle_positions[j])
                if d <= self.comm_range:
                    edges[(i, j)] = d
                    edges[(j, i)] = d
        return edges

    def _select_anchors(self):
        """Randomly select anchor nodes."""
        return {node: self.vehicle_positions[node] for node in random.sample(range(self.num_vehicles), self.num_anchors)}

    def _distance(self, pos1, pos2):
        """Calculate Euclidean distance."""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def ensure_full_connectivity(self):
        """Ensure all nodes have at least 3 neighbors."""
        for node in range(self.num_vehicles):
            while len(self.get_neighbors(node)) < 3:
                potential_neighbors = list(set(range(self.num_vehicles)) - {node} - set(self.get_neighbors(node)))
                if not potential_neighbors:
                    print(f"⚠️ Node {node} has no possible connections left.")
                    break
                new_neighbor = random.choice(potential_neighbors)
                distance = self._distance(self.vehicle_positions[node], self.vehicle_positions[new_neighbor])
                self.edges[(node, new_neighbor)] = distance
                self.edges[(new_neighbor, node)] = distance

    def get_neighbors(self, node):
        """Return a list of neighbors for a given node."""
        return [n for n in range(self.num_vehicles) if (node, n) in self.edges or (n, node) in self.edges]

    def get_edge_weights(self):
        """Return edge weights (distances)."""
        return self.edges

    def get_anchor_positions(self):
        """Return the positions of anchor nodes."""
        return self.anchors

    def get_distances(self):
        """Return distance constraints."""
        return self.edges

    def plot_network(self, output_file=None):
        """Visualize the vehicle network with positions and connections."""
        plt.figure(figsize=(10, 10))
        ax = plt.gca()

        # Plot edges (vehicle connections)
        for (i, j), distance in self.edges.items():
            x1, y1 = self.vehicle_positions[i]
            x2, y2 = self.vehicle_positions[j]
            plt.plot([x1, x2], [y1, y2], 'gray', linestyle='--', linewidth=0.7, alpha=0.5)

        # Plot vehicle positions
        for node, pos in self.vehicle_positions.items():
            color = 'blue' if node in self.anchors else 'green'
            label = "Anchor" if node in self.anchors else "Vehicle"
            plt.scatter(pos[0], pos[1], color=color, label=label if node == list(self.vehicle_positions.keys())[0] else "")
            plt.text(pos[0], pos[1], str(node), fontsize=9, color='black', ha='right')

        # Draw distance constraints as circles
        for anchor, pos in self.anchors.items():
            circle = plt.Circle(pos, self.comm_range, color='blue', fill=False, linestyle='dashed', alpha=0.3, label="Comm Range" if anchor == list(self.anchors.keys())[0] else "")
            ax.add_patch(circle)

        plt.legend()
        plt.title("Network Topology")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.grid()

        if output_file:
            plt.savefig(output_file)
        plt.show()
