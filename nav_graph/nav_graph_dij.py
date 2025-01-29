import json
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

class NavGraph:
    def __init__(self, graph_path, z_value=None):
        """Initialize the NavGraph class."""
        self.graph = self.load_graph(graph_path)
        if z_value is not None:
            self.adjust_z_coordinate(z_value)

    def load_graph(self, graph_path):
        """Load the navigation graph from a JSON file."""
        with open(graph_path, "r") as f:
            graph_data = json.load(f)
        return nx.node_link_graph(graph_data)

    def find_closest_node(self, point):
        """Find the closest node in the graph to a given point."""
        closest_node = None
        min_dist = float("inf")
        for node, data in self.graph.nodes(data=True):
            pos = np.array(data["pos"])
            dist = np.linalg.norm(pos - np.array(point))
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node

    def compute_trajectory(self, start_point, end_point):
        """Compute the trajectory between the start and end points."""
        # Find closest nodes to the start and end points
        start_node = self.find_closest_node(start_point)
        end_node = self.find_closest_node(end_point)

        # Compute shortest path in the graph
        try:
            path = nx.shortest_path(self.graph, source=start_node, target=end_node, weight="dist")
            trajectory = [self.graph.nodes[node]["pos"] for node in path]

            # Add the extra segment from start_point to start_node
            start_node_pos = np.array(self.graph.nodes[start_node]["pos"])
            trajectory.insert(0, start_point)  # Prepend start_point
            trajectory[1] = list(start_node_pos)  # Replace the first graph node with start_node position

            # Add the extra segment from end_node to end_point
            end_node_pos = np.array(self.graph.nodes[end_node]["pos"])
            trajectory.append(end_point)  # Append end_point
            trajectory[-2] = list(end_node_pos)  # Replace the last graph node with end_node position

            return trajectory
        except nx.NetworkXNoPath:
            print("No path found between the start and end points.")
            return None

    def visualize_trajectory(self, trajectory, save_path):
        """Visualize the trajectory and graph on a 2D plane."""
        plt.figure(figsize=(10, 8))

        # Plot the graph nodes and edges
        for edge in self.graph.edges:
            pos1 = np.array(self.graph.nodes[edge[0]]["pos"][:2])  # Only X, Y
            pos2 = np.array(self.graph.nodes[edge[1]]["pos"][:2])  # Only X, Y
            plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], color="gray", linestyle="--", linewidth=0.5)

        for node, data in self.graph.nodes(data=True):
            pos = np.array(data["pos"][:2])  # Only X, Y
            plt.scatter(pos[0], pos[1], color="blue", s=10, label="Graph Nodes" if node == 0 else "")

        # Plot the trajectory
        if trajectory:
            trajectory_points = np.array(trajectory)[:, :2]  # Only X, Y
            plt.plot(trajectory_points[:, 0], trajectory_points[:, 1], color="red", linewidth=2, label="Trajectory")
            plt.scatter(trajectory_points[:, 0], trajectory_points[:, 1], color="red", s=20)

            # Highlight start_point and end_point
            plt.scatter(trajectory_points[0, 0], trajectory_points[0, 1], color="green", s=50, label="Start Point")
            plt.scatter(trajectory_points[-1, 0], trajectory_points[-1, 1], color="orange", s=50, label="End Point")

            # Highlight start_node and end_node connections
            plt.plot(
                [trajectory_points[0, 0], trajectory_points[1, 0]],
                [trajectory_points[0, 1], trajectory_points[1, 1]],
                color="green", linestyle="--", linewidth=1, label="Start Connection"
            )
            plt.plot(
                [trajectory_points[-2, 0], trajectory_points[-1, 0]],
                [trajectory_points[-2, 1], trajectory_points[-1, 1]],
                color="orange", linestyle="--", linewidth=1, label="End Connection"
            )

        # Add labels
        plt.title("Trajectory Visualization")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid()
        plt.axis("equal")

        # Save the figure to the specified path
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        plt.close()
        print(f"Trajectory visualization saved to {save_path}")

    def adjust_z_coordinate(self, z_value):
        """Adjust the Z-coordinate of all nodes in the graph."""
        for node, data in self.graph.nodes(data=True):
            if 'pos' in data:
                x, y, _ = data['pos']
                data['pos'] = (x, y, z_value)

# Example usage
# graph = NavGraph("graph.json")
# trajectory = graph.compute_trajectory([0, 0, 0], [10, 10, 0])
# graph.visualize_trajectory(trajectory, "trajectory.png")
