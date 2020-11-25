def get_faces(edges, graph):
	edgeset = set(edges)

	faces = []
	path = [edges[0]]
	edgeset.discard(edges[0]);

	while (len(edgeset) > 0):
		neighbors = graph[path[-1][-1]]
		next_node = neighbors[(neighbors.index(path[-1][-2]) + 1) % (len(neighbors))]
		tup = (path[-1][-1], next_node)
		if tup == path[0]:
			faces.append(path)
			path = []
			for edge in edgeset:
				path.append(edge)
				edgeset.discard(edge)
				break
		else:
			path.append(tup)
			edgeset.discard(tup)

	if (len(path) != 0): faces.append(path)
	return iter(faces)

import sys
import ast

if __name__=="__main__":
	# graph = []
	# with open(sys.argv[1]) as file:
	# 	text = file.read()
	# 	graph = ast.literal_eval(text)

	edges = []
	graph = [[1, 5], [0,2], [6,1,7,3], [6, 2, 8, 4], [3, 5], [4, 0], [2, 3], [2, 8], [7, 3]]
	for node, adj in enumerate(graph):
		for neighbor in adj:
			if node < neighbor:
				edges.append((node, neighbor))
			else:
				edges.append((neighbor, node))

	# (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0), (2, 7), (7, 8), (8, 3), (3, 6), (6, 2)]

	faces = get_faces(edges, graph)
	for face in faces:
		print(face)