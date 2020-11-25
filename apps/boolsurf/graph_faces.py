def get_faces(graph):
	edges = []
	for node, adj in enumerate(graph):
		for neighbor in adj:
			if node < neighbor:
				edges.append((node, neighbor))
			else:
				edges.append((neighbor, node))

	edgeset = set(edges)

	faces = []
	path = [edges[0]]
	edgeset.discard(edges[0]);

	while (len(edgeset) > 0):
		neighbors = graph[path[-1][-1]]
		last_node = path[-1][1]
		next_node = neighbors[(neighbors.index(path[-1][0]) + 1) % (len(neighbors))]
		tup = (last_node, next_node)
		if tup == path[0]:
			faces.append(path)
			# clear path
			path = []
			for edge in edgeset:
				path.append(edge)
				edgeset.discard(edge)
				break
		else:
			path.append(tup)
			edgeset.discard(tup)

	if len(path) != 0: faces.append(path)
	return iter(faces)

import sys
import ast

if __name__=="__main__":
	graph = [[1, 5], [0,2], [6,1,7,3], [6, 2, 8, 4], [3, 5], [4, 0], [2, 3], [2, 8], [7, 3]]

	if len(sys.argv) > 1:
		with open(sys.argv[1]) as file:
			text = file.read()
			graph = ast.literal_eval(text)


	faces = get_faces(graph)
	for face in faces:
		print(face)