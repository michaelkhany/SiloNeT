"""
Network Localization Simulation

This script simulates vehicle localization using offline localization strategies. 
It initializes a vehicle network, retrieves distance constraints, and applies an 
offline localization algorithm to estimate unknown positions.

Key Features:
- Uses a network simulator to generate a vehicle network with known anchor nodes.
- Supports different offline localization strategies by modifying the import section.
- Computes localization accuracy by comparing estimated positions with actual positions.
- Provides a visualization of network topology and localization results.

Important:
- Only modify the configuration section (e.g., change the offline localization strategy or number of nodes in the network, etc.).
- Do not alter the rest of the script unless necessary, as it ensures correct evaluation 
  and visualization of localization results.
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from simulator.network_simulator import NetworkSimulator

# ------------------------------------------------------------------------------------------------------
# Change this class library to try different offline localization strategies vvvvvvvvvvvvvvvvvvvvvvvvvvv
# from student_algorithms.trilateration_offline_localization import OfflineLocalization
from student_algorithms.beliefPropagationLocalization import OfflineLocalization

# Initialize the network simulator
sim = NetworkSimulator(num_vehicles=10, comm_range=70, area_size=100, num_anchors=3)
sim.plot_network(output_file="output/network_topology.png")

# # Threshold for correct localization
# THRESHOLD = 15.0

# ------------------------------------------------------------------------------------------------------

# Helper function to calculate Euclidean distance
def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# Retrieve network information
anchor_positions = sim.get_anchor_positions()
distances = sim.get_distances()
actual_positions = sim.vehicle_positions

# Print initial network setup
print("\n=== Network Configuration ===")
print(f"Anchor Nodes: {anchor_positions}")
print(f"All Actual Positions: {actual_positions}")
print(f"Distance Constraints: {distances}")

# Initialize the offline localization algorithm
localization = OfflineLocalization()

# Estimate positions using the algorithm
print("\n=== Running Offline Localization ===")
estimated_positions = localization.locate_other_vehicles(
    known_positions=anchor_positions, distances=distances
)

# Analyze localization accuracy
# Define a threshold for correct localization (e.g., within 2 units)
ACCURACY_THRESHOLD = 2.0  

# Compare estimated positions with actual positions
matched = 0
total_estimated = len(estimated_positions)

print("\n=== Distance-Based Matching ===")
for node_id, actual_pos in actual_positions.items():
    if node_id in estimated_positions:
        estimated_pos = estimated_positions[node_id]
        distance = np.linalg.norm(np.array(actual_pos) - np.array(estimated_pos))
        
        print(f"Node {node_id}: Actual {actual_pos} -> Estimated {estimated_pos}, Distance: {distance:.2f}")
        
        if distance <= ACCURACY_THRESHOLD:
            matched += 1  # Count as correctly localized

# Compute accuracy percentage
accuracy = (matched / total_estimated) * 100 if total_estimated > 0 else 0
print(f"\n=== Localization Accuracy: {accuracy:.2f}% ===\n")

# Visualization
def visualize_localization(actual_positions, estimated_positions, distance_constraints, output_file=None):
    """Visualize actual vs. estimated positions along with distance constraints."""
    
    plt.figure(figsize=(10, 10))
    
    # Convert actual and estimated positions to numpy arrays
    actual_positions_arr = np.array(list(actual_positions.values()))
    estimated_positions_arr = np.array(list(estimated_positions.values()))
    
    # Plot known anchor positions in **blue**
    for node, pos in actual_positions.items():
        plt.scatter(pos[0], pos[1], color='blue', label="Anchor Node" if node == list(actual_positions.keys())[0] else "")
        plt.text(pos[0], pos[1], str(node), fontsize=9, color='black', ha='right')

    # Plot estimated positions in **red**
    for node, pos in estimated_positions.items():
        plt.scatter(pos[0], pos[1], color='red', label="Estimated Node" if node == list(estimated_positions.keys())[0] else "")
        plt.text(pos[0], pos[1], str(node), fontsize=9, color='black', ha='left')

    # Draw distance constraints as **dashed circles**
    for (i, j), d in distance_constraints.items():
        if i in actual_positions and j in estimated_positions:
            center = actual_positions[i]  # The known node
            circle = plt.Circle(center, d, color='gray', fill=False, linestyle='dashed', alpha=0.5)
            plt.gca().add_patch(circle)

    # Highlight **overlapping positions** (correctly estimated)
    overlap_points = [node for node in estimated_positions if np.allclose(actual_positions[node], estimated_positions[node], atol=10)]
    for node in overlap_points:
        plt.scatter(*actual_positions[node], color='purple', marker='o', s=100, label="Correctly Estimated" if node == overlap_points[0] else "", edgecolors='black')

    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.legend()
    plt.grid()
    plt.title("Network Localization: Actual vs Estimated Positions")

    if output_file:
        plt.savefig(output_file)
    plt.show()

visualize_localization(actual_positions, estimated_positions, distances, output_file="output/localization_result.png")

# Debugging: Print final estimated positions
print("\n=== Final Estimated Positions ===")
for car, estimated_pos in estimated_positions.items():
    print(f"Node {car}: Estimated Position: {estimated_pos}")

# # Overlay actual vs. estimated positions with connections
# plt.figure(figsize=(10, 10))
# edge_weights = sim.get_edge_weights()

# # Plot edges between vehicles
# for (i, j), weight in edge_weights.items():
#     x1, y1 = actual_positions[i]
#     x2, y2 = actual_positions[j]
#     plt.plot([x1, x2], [y1, y2], 'gray', linestyle='--', linewidth=0.5)

# # Actual positions
# for node, pos in actual_positions.items():
#     plt.scatter(pos[0], pos[1], color='blue', label='Actual' if node == 0 else "")
#     plt.text(pos[0], pos[1], str(node), fontsize=9, color='black')

# # Estimated positions
# for node, pos in estimated_positions.items():
#     plt.scatter(pos[0], pos[1], color='red', label='Estimated' if node == 0 else "")
#     plt.text(pos[0], pos[1], str(node), fontsize=9, color='black')

# plt.legend(['Edges', 'Actual Positions', 'Estimated Positions'])
# plt.title("Network Localization: Actual vs Estimated Positions")
# plt.show()
