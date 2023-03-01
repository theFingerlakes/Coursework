import networkx as net
from single_agent_planner import build_constraint_table, is_constrained, get_path, move


def construct_MDD_for_agent(my_map, agent, start_loc, goal_loc, h_values, cost, constraints):
	MDD = net.DiGraph()
	h_value = h_values[start_loc]
	open_list = []
	constraintTable = build_constraint_table(constraints, agent)

	root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep':0}
	open_list.append(root)

	while len(open_list) > 0:
		curr = open_list.pop(0)
		if curr['timestep'] == cost:
			if curr['loc'] == goal_loc:
				path = get_path(curr)
				for i in range(len(path) - 1):
					MDD.add_edge((path[i], i), (path[i+1], i+1))
			continue

		for dir in range(5):
			child_loc = move(curr['loc'], dir)
			if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
			or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
				continue
			if my_map[child_loc[0]][child_loc[1]]:
				continue
			if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraintTable):
				continue
			if curr['g_val'] + h_values[child_loc] + 1 > cost:
				continue

			child = {'loc': child_loc,
					'g_val': curr['g_val'] + 1,
					'h_val': h_values[child_loc],
					'parent': curr,
					'timestep': curr['timestep'] + 1}
			open_list.append(child)
	return MDD  

def construct_MDD(my_map, num_of_agents,starts, goals, h_values, paths, constraints):
	MDD = []
	for i in range(num_of_agents):
		MDD.append(construct_MDD_for_agent(my_map, i, starts[i], goals[i], h_values[i], len(paths[i]) - 1, constraints)) 
	return MDD


def reconstruct_MDD(MDD, start_loc):
	new_MDD = {}
	locations = net.single_source_shortest_path_length(MDD, (start_loc, 0)) 
	for loc, depth in locations.items():
		if depth not in new_MDD:
			new_MDD[depth] = []
		new_MDD[depth].append(loc[0])
	return new_MDD

def updateMDD(MDD, agent, start_loc, goal_loc, cost, constraints):
	constraintTable = build_constraint_table(constraints, agent)
	MDD_copy = MDD.copy()
	recons_MDD = reconstruct_MDD(MDD, start_loc)
	for timestep, locations in recons_MDD.items():	
		if locations[0] == goal_loc:	
			break
		else: 
			for curr_loc in locations:
				for next_loc in list(MDD_copy.successors((curr_loc,timestep))):
					if is_constrained(curr_loc, next_loc[0], timestep+1, constraintTable):
						MDD_copy.remove_edge((curr_loc, timestep), next_loc)
	deleted_nodes = []
	for node in net.nodes(MDD_copy):
		if node == (start_loc, 0):
			continue
		elif node != (goal_loc, cost):
			if len(list(MDD_copy.predecessors(node))) == 0 \
			or len(list(MDD_copy.successors(node))) == 0:
				deleted_nodes.append(node)
	MDD_copy.remove_nodes_from(deleted_nodes)		
	return MDD_copy


def merge_MDD(MDD1, start1, goal1, MDD2, start2, goal2):
	len1 = len(reconstruct_MDD(MDD1, start1))
	len2 = len(reconstruct_MDD(MDD2, start2))
	MDD1_copy = MDD1.copy()
	MDD2_copy = MDD2.copy()
	if len1 > len2:
		edges = []
		for i in range(len2, len1):
			edges.append(((goal2, i-1), (goal2, i)))
		MDD2_copy.add_edges_from(edges)
	elif len1 < len2:
		edges = []
		for i in range(len1, len2):
			edges.append(((goal1, i-1), (goal1, i)))
		MDD1_copy.add_edges_from(edges)
	joint_MDD = {0:[(start1, start2)]}
	for i in range(max(len1, len2) - 1):
		joint_MDD[i+1] = []
		for pair in joint_MDD[i]:
			successor1 = [successor for successor, _ in list(MDD1_copy.successors((pair[0], i)))]
			successor2 = [successor for successor, _ in list(MDD2_copy.successors((pair[1], i)))]
			cross_product = [(x, y) for x in successor1 for y in successor2 if x != y]
			for new_pair in cross_product:
				if new_pair not in joint_MDD[i+1]:
					joint_MDD[i+1].append(new_pair)
		if len(joint_MDD[i+1]) == 0:
			return joint_MDD, max(len1, len2)-1
			
	return joint_MDD, max(len1, len2)-1



