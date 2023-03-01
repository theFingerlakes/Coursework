import networkx as net
from mvc import get_MVC
from mdd import reconstruct_MDD, merge_MDD


def cardinal_detection(MDD1, start1, MDD2, start2):
	MDD1 = reconstruct_MDD(MDD1, start1)
	MDD2 = reconstruct_MDD(MDD2, start2)
	cost = min(len(MDD1), len(MDD2))
	for timestep in range(cost):
		if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
		and MDD1[timestep][0] == MDD2[timestep][0]:
			return True
		if timestep < cost - 1:
			if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
			and len(MDD1[timestep+1]) == 1 and len(MDD2[timestep+1]) == 1 \
			and MDD1[timestep][0] == MDD2[timestep+1][0] \
			and MDD1[timestep+1][0] == MDD2[timestep][0]:
				return True
	return False

def construct_conflict_graph(num_of_agents, MDD, starts):
	conflict_graph = net.Graph()
	for i in range(num_of_agents - 1):
		for j in range(i + 1, num_of_agents):
			if cardinal_detection(MDD[i], starts[i], MDD[j], starts[j]):
				conflict_graph.add_nodes_from([i, j])
				conflict_graph.add_edge(i, j)
	return conflict_graph



def compute_CG(MDD, num_of_agents, starts):
	conflict_graph = construct_conflict_graph(num_of_agents, MDD, starts)
	h_value = get_MVC(conflict_graph) # construct confict_graph
	return h_value


def cardinal_detection(MDD1, start1, MDD2, start2):
	MDD1 = reconstruct_MDD(MDD1, start1)
	MDD2 = reconstruct_MDD(MDD2, start2)
	cost = min(len(MDD1), len(MDD2))
	for timestep in range(cost):
		if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
		and MDD1[timestep][0] == MDD2[timestep][0]:
			return True
		if timestep < cost - 1:
			if len(MDD1[timestep]) == 1 and len(MDD2[timestep]) == 1 \
			and len(MDD1[timestep+1]) == 1 and len(MDD2[timestep+1]) == 1 \
			and MDD1[timestep][0] == MDD2[timestep+1][0] \
			and MDD1[timestep+1][0] == MDD2[timestep][0]:
				return True
	return False



def isDependent(joint_MDD, goal1, goal2, max_level):
    if max_level in joint_MDD:
        if (goal1, goal2) in joint_MDD[max_level]:
            return False
    return True

def construct_dependency_graph(num_of_agents, MDD, starts, goals):
    dependency_graph = net.Graph()
    for i in range(num_of_agents - 1):
        for j in range(i + 1, num_of_agents):
            joint_MDD, max_level = merge_MDD(
                MDD[i], starts[i], goals[i], MDD[j], starts[j], goals[j])
            if isDependent(joint_MDD, goals[i], goals[j], max_level) \
                    or cardinal_detection(MDD[i], starts[i], MDD[j], starts[j]):
                dependency_graph.add_nodes_from([i, j])
                dependency_graph.add_edge(i, j)
    return dependency_graph



def compute_DG(MDD, num_of_agents, starts, goals):
	dependency_graph = construct_dependency_graph(num_of_agents, MDD, starts, goals)
	h_value = get_MVC(dependency_graph)
	return h_value