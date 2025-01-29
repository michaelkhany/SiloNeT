"""
Offline Localization using Trilateration

This class estimates unknown positions based on distances to known points 
using trilateration. It is useful for applications such as GPS, robotics, 
and sensor networks where only relative distances are available.

Key Features:
- Computes circle intersections to determine possible locations.
- Uses trilateration to estimate unknown positions.
- Dynamically locates multiple unknown nodes through an iterative process.

This method is commonly used in navigation and positioning systems, and 
adjusting the tolerance value can improve accuracy depending on the dataset.
"""

import numpy as np
class OfflineLocalization:
    def __init__(self, tolerance=1e-2):
        self.tolerance = tolerance

    def circle_intersection(self, p1, r1, p2, r2):
        """Computes intersection points of two circles."""
        p1, p2 = np.array(p1), np.array(p2)
        d = np.linalg.norm(p1 - p2)

        if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
            return None, None  # No valid intersection or circles coincide

        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = np.sqrt(max(0, r1**2 - a**2))
        mid = p1 + a * (p2 - p1) / d
        offset = h * np.array([- (p2[1] - p1[1]) / d, (p2[0] - p1[0]) / d])

        return mid + offset, mid - offset

    def trilaterate(self, p1, p2, p4, d1, d2, d4):
        """Estimates a position using trilateration from three known points."""
        p1, p2, p4 = np.array(p1), np.array(p2), np.array(p4)

        print(f"🔵 Trilaterating with points: P1={p1}, P2={p2}, P4={p4}")
        print(f"Distances: d1={d1}, d2={d2}, d4={d4}")

        p3a, p3b = self.circle_intersection(p1, d1, p2, d2)
        if p3a is None or p3b is None:
            print("❌ No valid intersection found!")
            return None

        print(f"✅ Possible solutions: P3a={p3a}, P3b={p3b}")

        d3a = np.linalg.norm(p4 - p3a)
        d3b = np.linalg.norm(p4 - p3b)

        if np.isclose(d3a, d4, atol=self.tolerance):
            print(f"✅ Choosing P3a: {p3a}")
            return tuple(p3a)
        elif np.isclose(d3b, d4, atol=self.tolerance):
            print(f"✅ Choosing P3b: {p3b}")
            return tuple(p3b)
        else:
            print("⚠️ Neither solution satisfies the distance constraint. Returning weighted midpoint as fallback.")
            return tuple((p3a + p3b) / 2)  # Return midpoint if both are invalid

    def locate_other_vehicles(self, known_positions, distances):
        """Dynamically estimates unknown vehicle positions one by one."""
        estimated_positions = {}
        unknown_nodes = set(node for edge in distances for node in edge) - set(known_positions.keys())

        max_iterations = len(unknown_nodes) * len(unknown_nodes)  # Prevent infinite loops
        iteration_count = 0

        while len(estimated_positions) < len(unknown_nodes):
            progress_made = False  # Track progress
            pending_nodes = []  # Store skipped nodes for later attempts

            for (id1, id2), d in distances.items():
                if id1 in known_positions and id2 not in known_positions:
                    # Identify known neighbors
                    neighbors = sorted(
                        [(k, distances.get((id2, k), distances.get((k, id2))))
                        for k in known_positions.keys() if (id2, k) in distances or (k, id2) in distances],
                        key=lambda x: x[1]
                    )

                    if len(neighbors) < 3:
                        pending_nodes.append(id2)  # Store for later attempt
                        continue

                    # Select three closest known neighbors
                    (p1_id, d1), (p2_id, d2), (p4_id, d4) = neighbors[:3]
                    p1, p2, p4 = known_positions[p1_id], known_positions[p2_id], known_positions[p4_id]
                    estimated_p = self.trilaterate(p1, p2, p4, d1, d2, d4)

                    if estimated_p is not None:
                        estimated_positions[id2] = estimated_p
                        known_positions[id2] = estimated_p  # Update known positions
                        progress_made = True
                        print(f"📍 Estimated position for node {id2}: {estimated_p}")

            iteration_count += 1

            if iteration_count > max_iterations:
                print("⚠️ Maximum iterations reached! Possible disconnected nodes.")
                break  # Stop if iterations exceed the limit

            if not progress_made and pending_nodes:
                print(f"🔄 Retrying pending nodes: {pending_nodes}")
            elif not progress_made:
                print("❌ No further progress can be made. Exiting...")
                break  # Stop if no new positions are estimated

        return estimated_positions
