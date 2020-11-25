def get_faces(edges, embedding):
	edgeset = set()
	for edge in edges:
		edge = list(edge)
		edgeset |= set([(edge[0], edge[1]), (edge[1], edge[0])])

	faces = []
	path = []
	for edge in edgeset:
		path.append(edge)
		edgeset -= set([edge])
		break

	while (len(edgeset) > 0):
		neighbors = embedding[path[-1][-1]]
		next_node = neighbors[(neighbors.index(path[-1][-2]) + 1) % (len(neighbors))]
		tup = (path[-1][-1], next_node)
		if tup == path[0]:
			faces.append(path)
			path = []
			for edge in edgeset:
				path.append(edge)
				edgeset -= set([edge])
				break
		else:
			path.append(tup)
			edgeset -= set([tup])

	if (len(path) != 0): faces.append(path)
	return iter(faces)


if __name__=="__main__":
	edges = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0), (2, 7), (7, 8), (8, 3), (3, 6), (6, 2)]
	embedding = {0: [1, 5], 1: [0,2], 2: [6,1,7,3], 3: [6, 2, 8, 4], 4: [3, 5], 5:[4, 0], 6: [2, 3], 7: [2, 8], 8: [7, 3]}

	faces = get_faces(edges, embedding)
	for face in faces:
		print(face)