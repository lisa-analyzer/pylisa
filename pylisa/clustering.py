from sknetwork.hierarchy import Paris
from sknetwork.visualization import svg_dendrogram
from sknetwork.ranking import PageRank
from sknetwork.data import from_adjacency_list
import numpy as np
from IPython.display import SVG
from sknetwork.clustering import Louvain, get_modularity
from sknetwork.hierarchy import Paris, cut_straight, dasgupta_score, tree_sampling_divergence
paris = Paris()
ids = {}
verticesNo = 0
edgesNo = 0
adj_matrix = None
adj_list = []
### PARSE FILE
with open("/Users/giacomozanatta/Projects/pylisa-ros/analysis/ros2-beginner/dir-graph-adj-matrix.txt") as f:
    lines = f.read().split('\n')
    found_ids = False
    found_graph = False
    cur_id = ""
    from_idx = -1
    to_idx = -1
    found_vno = False
    matrix_initialized = False
    changed_from = True
    for line in lines:
        if line == "§§§ IDs §§§":
            found_ids = True
            print("Found ids!")
        else:
            if found_ids and not found_graph:
                if line == "§§§ MATRIX §§§":
                    print("found matrix!")
                    found_graph = True
                else:
                    id = line.split(" ")
                    ids[id[0]] = id[1]
                    print(line)
            else:
                if found_ids and found_graph:
                    if line.startswith("#vertices"):
                        print("found verticesNo!")
                        verticesNo = line.split(" ")[1]
                    else:
                        if line.startswith("#edges"):
                            print("found edgesNo!")
                            edgesNo = line.split(" ")[1]
                        else:
                            if not matrix_initialized:
                                print("MATRIX")
                                adj_list = []
                                for i in range(0, int(verticesNo)):
                                    adj_list.append([])
                                matrix_initialized = True
                            id = line.split(" ")
                            if adj_list[int(id[0])] == None:
                                adj_list[int(id[0])] = []
                            adj_list[int(id[0])].append(id[1])
### COMPUTE

print("No. vertices: " + verticesNo)
print("No. edges: " + edgesNo)
print("Adj list: " + str(adj_list))

from sknetwork.hierarchy import Paris
from sknetwork.visualization import svg_graph, svg_bigraph
from sknetwork.visualization import svg_dendrogram

adj_matrix_from_edges = from_adjacency_list(adj_list, directed = True)
n = adj_matrix_from_edges.shape[0]
print(adj_matrix_from_edges)
names = []
for key in ids:
    names.append(ids[key])
image = svg_graph(adj_matrix_from_edges, directed = True, names=names)
f = open("graph.svg", "w")
f.write(image)
f.close()
SVG(image)
# Instantiate and fit the Paris algorithm
louvain = Louvain()
labels = louvain.fit_predict(adj_matrix_from_edges)
dendogram = Paris().fit_predict(adj_matrix_from_edges)
labelsP = cut_straight(dendogram, n_clusters = 3)

image = svg_graph(adj_matrix_from_edges, directed = True, labels=labels, names=names)
#image = svg_dendrogram(labels, names=names)
f = open("graph-cluster-louvain.svg", "w")
f.write(image)
f.close()

image = svg_graph(adj_matrix_from_edges, directed = True, labels=labelsP, names=names)
f = open("graph-cluster-paris.svg", "w")
f.write(image)
f.close()