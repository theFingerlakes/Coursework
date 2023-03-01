import networkx as net
import numpy as np
import math


def get_MVC(G):
	upperbound = net.number_of_nodes(G)
	C = []
	MVC = EMVC(G, upperbound, C)
	return MVC


def EMVC(G, upperbound, C):
	if net.is_empty(G):
		return len(C)
	cliques = get_disjoint_cliques(G)
	ClqLB = 0
	for clique in cliques:
		ClqLB += len(clique) - 1
	H = G.copy()
	num_of_edges = net.number_of_edges(G)
	nodes = []
	degrees = []
	for degree in G.degree():
		nodes.append(degree[0])
		degrees.append(degree[1])
	DegLB = compute_DegLB(H, nodes, degrees, num_of_edges)
	if len(C) + max(DegLB, ClqLB) >= upperbound:
		return upperbound
	
	largest_index = np.argmax(degrees)
	vertex = nodes[largest_index]

	neighbors = [n for n in G.neighbors(vertex)]
	A = G.copy()
	A.remove_nodes_from(neighbors)
	A.remove_node(vertex)
	B = G.copy()
	B.remove_node(vertex)
	c1 = EMVC(A, upperbound, C + neighbors)
	c2 = EMVC(B, min(upperbound, c1), C + [vertex])
	return min(c1, c2)

def compute_DegLB(H, nodes, degrees, num_of_edges):
	i = 0
	total_degrees = 0
	while total_degrees < num_of_edges:

		largest_index = np.argmax(degrees)
		total_degrees += degrees[largest_index]
		H.remove_node(nodes[largest_index])
		degrees.remove(degrees[largest_index])
		nodes.remove(nodes[largest_index])
		i += 1
	num_of_edges_afterRemove = net.number_of_edges(H)
	max_degree_afterRemove = max(degrees)
	DegLB = math.floor(i+num_of_edges_afterRemove/max_degree_afterRemove)
	return DegLB	

def get_disjoint_cliques(G):
	disjoint_cliques = []
	existing_nodes = []
	cliques = list(net.find_cliques(G))
	cliques.sort(key = len, reverse = True)
	for clique in cliques:
		if len(disjoint_cliques) == 0:
			disjoint_cliques.append(clique)
			existing_nodes = existing_nodes + clique
		else:
			if len(set(clique).intersection(set(existing_nodes))) == 0:
				disjoint_cliques.append(clique)
				existing_nodes = existing_nodes + clique
	if net.number_of_nodes(G) == len(existing_nodes):
		return disjoint_cliques
	else:
		nodes = [node for node in net.nodes(G) if node not in existing_nodes]
		subgraph = G.subgraph(nodes)
		disjoint_cliques = disjoint_cliques + get_disjoint_cliques(subgraph)
		return disjoint_cliques


def compute_EWMVC(dependency_graph):
    G = dependency_graph.copy()
    connected_subgraphs = [G.subgraph(c) for c in net.connected_components(G)]
    EWMVC = 0
    for component in connected_subgraphs:
        EWMVC += compute_EWMVC_component(component)
    return EWMVC


def compute_EWMVC_component(graph):
    G = graph.copy()
    num_of_nodes = net.number_of_nodes(G)
    nodes = net.nodes(G)
    possible_values = {}
    for node in nodes:
        possible_values[node] = []
        maximum = min([G.edges[edge]['weight'] for edge in G.edges(node)])
        for i in range(maximum + 1):
            possible_values[node].append(i)
    value_list = {}
    best = float('inf')
    best = bnb(possible_values, value_list, nodes, G, best)
    return best


def bnb(possible_values, value_list, nodes, G, best):
    if len(value_list) == len(possible_values):
        cost = sum([value for _, value in value_list.items()])
        return cost
    else:
        value_list_copy = value_list.copy()
        unassigned_nodes = [
            node for node in nodes if node not in value_list_copy]
        node = unassigned_nodes[0]
        for value in possible_values[node]:
            if isViolated(value_list_copy, G, node, value):
                continue
            value_list_copy[node] = value
            cost = bnb(possible_values, value_list_copy, nodes, G, best)
            if cost < best:
                best = cost
        return best


def isViolated(value_list, G, node, value):
    for key, val in value_list.items():
        if (key, node) in G.edges():
            if val + value < G.edges[key, node]['weight']:
                return True
    return False