# UEBUNG-3-GRAPHEN-
Task: Shortest Path in the Viennese Transport Network
You are given a graph with weighted edges. Develop and implement an algorithm that finds the most cost-effective path between two nodes.

Finding the shortest path between two nodes in a graph is a fundamental problem in graph theory, and it has numerous real-world applications, including route planning in transportation networks. In this particular task, we are dealing with the Viennese transport network.

To solve this problem, we can employ various graph traversal algorithms, such as Dijkstra's algorithm or the A* search algorithm. These algorithms consider the weights of the edges to determine the shortest path between two given nodes.

Dijkstra's algorithm, for example, starts at the initial node and explores the neighboring nodes, assigning tentative distances to them. It iteratively selects the node with the smallest tentative distance and continues exploring its neighbors until it reaches the destination node or exhausts all possible paths. The algorithm guarantees finding the shortest path in terms of accumulated edge weights.

The implementation of this algorithm involves representing the transport network as a graph data structure, where nodes represent locations (e.g., intersections, stations) and edges represent connections between them. Each edge has a weight that corresponds to the cost or distance associated with traveling from one location to another.

By applying the shortest path algorithm to the Viennese transport network, we can efficiently determine the most cost-effective route between any two given locations within the network. This information can be valuable for various purposes, such as optimizing transportation logistics, providing accurate travel directions, or estimating travel times.

In conclusion, the task at hand involves developing and implementing an algorithm that can find the most cost-effective path between two nodes in the Viennese transport network. By leveraging graph traversal algorithms, we can efficiently solve this problem and provide valuable insights for route planning and optimization in Vienna's transportation system.
