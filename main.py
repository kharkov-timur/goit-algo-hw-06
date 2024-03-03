import networkx as nx
import matplotlib.pyplot as plt
import heapq


# Пошук у глибину (DFS)
def dfs_paths(graph, start, goal, path=[]):
    path = path + [start]
    if start == goal:
        return [path]
    if start not in graph:
        return []
    paths = []
    for node in graph[start]:
        if node not in path:
            new_paths = dfs_paths(graph, node, goal, path)
            for new_path in new_paths:
                paths.append(new_path)
    return paths


# Пошук у ширину (BFS)
def bfs_paths(graph, start, goal):
    queue = [(start, [start])]
    while queue:
        (node, path) = queue.pop(0)
        for next_node in set(graph[node]) - set(path):
            if next_node == goal:
                yield path + [next_node]
            else:
                queue.append((next_node, path + [next_node]))


# Алгоритм Дейкстри
def dijkstra(graph, start):
    distances = {node: float("inf") for node in graph}
    distances[start] = 0
    queue = [(0, start)]

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, edge_data in graph[current_node].items():
            weight = edge_data["weight"]
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))

    return distances


# Створення графа
G = nx.Graph()
G.add_nodes_from(["User1", "User2", "User3", "User4", "User5"])
edges = [
    ("User1", "User2", {"weight": 1}),
    ("User1", "User3", {"weight": 2}),
    ("User2", "User3", {"weight": 1}),
    ("User2", "User4", {"weight": 2}),
    ("User3", "User4", {"weight": 1}),
    ("User4", "User5", {"weight": 3}),
]
G.add_edges_from(edges)

# Візуалізація графа
plt.figure(figsize=(8, 6))
nx.draw(
    G,
    with_labels=True,
    node_size=1000,
    node_color="skyblue",
    font_size=12,
    font_weight="bold",
)
plt.title("Соціальна мережа")
plt.show()


# Аналіз основних характеристик графа
print("1. Аналіз основних характеристик графа:")
print("\tКількість вершин:", G.number_of_nodes())
print("\tКількість ребер:", G.number_of_edges())
print("\tСписок вершин:", list(G.nodes()))
print("\tСписок ребер:", list(G.edges()))
print("\tСтупінь вершин:", dict(G.degree()))


print("\n2. Пошук у графах, алгоритми DFS і BFS:")
# Знаходження шляхів за допомогою DFS
print("\tШляхи DFS:")
for path in dfs_paths(G, "User1", "User5"):
    print(f"\t\t{path}")

# Знаходження шляхів за допомогою BFS
print("\tШляхи BFS:")
for path in bfs_paths(G, "User1", "User5"):
    print(f"\t\t{path}")


# Застосуємо алгоритм Дейкстри для знаходження найкоротших шляхів від вершини 'User1' до всіх інших вершин
shortest_paths = {node: dijkstra(G, node) for node in G.nodes()}

print(
    "\n3. Застосуємо алгоритм Дейкстри для знаходження найкоротших шляхів від вершини 'User1' до всіх інших вершин"
)
# Виведемо результати
for node in shortest_paths:
    print("\tНайкоротші шляхи від вершини", node, "до всіх інших вершин:")
    for destination, distance in shortest_paths[node].items():
        print("\t\tШлях до", destination, ":", distance)
