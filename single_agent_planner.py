import heapq
import networkx as nx

def move(loc, dir):
	directions = [(0, 0), (0, -1), (1, 0), (0, 1), (-1, 0)] 
	return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
	rst = 0
	for path in paths:
		rst += len(path) - 1
	return rst


def compute_heuristics(my_map, goal):
	open_list = []
	closed_list = dict()
	root = {'loc': goal, 'cost': 0}
	heapq.heappush(open_list, (root['cost'], goal, root))
	closed_list[goal] = root
	while len(open_list) > 0:
		(cost, loc, curr) = heapq.heappop(open_list)
		for dir in range(5):
			child_loc = move(loc, dir)
			child_cost = cost + 1
			if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
			   or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
			   continue
			if my_map[child_loc[0]][child_loc[1]]:
				continue
			child = {'loc': child_loc, 'cost': child_cost}
			if child_loc in closed_list:
				existing_node = closed_list[child_loc]
				if existing_node['cost'] > child_cost:
					closed_list[child_loc] = child
					heapq.heappush(open_list, (child_cost, child_loc, child))
			else:
				closed_list[child_loc] = child
				heapq.heappush(open_list, (child_cost, child_loc, child))

	h_values = dict()
	for loc, node in closed_list.items():
		h_values[loc] = node['cost']
	return h_values


def build_constraint_table(constraints, agent):

	table = {}
	for constraint in constraints:
		if constraint['agent'] == agent:
			ts = constraint['timestep']
			if ts not in table:
				table[constraint['timestep']] = []
			table[ts].append(constraint)
		elif constraint['agent'] == 'goal':
			if 'goal' not in table:
				table['goal'] = []
			table['goal'].append(constraint)
		elif constraint['positive'] is True:
			constraint['agent'] = agent
			constraint['positive'] = False
			constraint['loc'].reverse()
			ts = constraint['timestep']
			if ts not in table:
				table[constraint['timestep']] = []
			table[ts].append(constraint)
	return table


def get_location(path, time):
	if time < 0:
		return path[0]
	elif time < len(path):
		return path[time]
	else:
		return path[-1]


def get_path(goal_node):
	path = []
	curr = goal_node
	while curr is not None:
		path.append(curr['loc'])
		curr = curr['parent']
	path.reverse()
	return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
	if 'goal' in constraint_table:
		for constraint in constraint_table['goal']:
			if next_loc == constraint['loc'][0] and next_time >= constraint['timestep']:
				return True
	if next_time in constraint_table:
		constraints = constraint_table[next_time]
		for constraint in constraints:
			if constraint['positive'] == False:
				if len(constraint['loc']) == 1:
					if constraint['loc'][0] == next_loc:
						return True
				else:
					if constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc:
						return True
			else:
				if len(constraint['loc']) == 1:
					if constraint['loc'][0] is not next_loc:
						return True
				else:
					if constraint['loc'][0] == curr_loc and constraint['loc'][1] is not next_loc:
						return True
	return False


def push_node(open_list, node):
	heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
	_, _, _, curr = heapq.heappop(open_list)
	return curr


def compare_nodes(n1, n2):
	"""Return true is n1 is better than n2."""
	return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
	""" my_map      - binary obstacle map
		start_loc   - start position
		goal_loc    - goal position
		agent       - the agent that is being re-planned
		constraints - constraints defining where robot should or cannot go at each timestep
	"""

	open_list = []
	closed_list = dict()
	earliest_goal_timestep = 0
	h_value = h_values[start_loc]
	constraintTable = build_constraint_table(constraints, agent)
	


	root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep':0}
	push_node(open_list, root)
	closed_list[(root['loc'], root['timestep'])] = root
	path_length = 0
	while len(open_list) > 0:
		curr = pop_node(open_list)
		goalConstraints = False
		for key in constraintTable.keys():
			if key != 'goal' and key > curr['timestep']:
				goal = {'loc': [goal_loc], 'agent': agent, 'timestep': key, 'positive': False}
				if goal in constraintTable[key]:
					goalConstraints = True
					break

		if curr['loc'] == goal_loc and not goalConstraints:
			return get_path(curr)

		for dir in range(5):
			child_loc = move(curr['loc'], dir)
			if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
			or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
				continue
			
			if my_map[child_loc[0]][child_loc[1]]:
				continue
			if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraintTable):
				continue
			
			child = {'loc': child_loc,
					'g_val': curr['g_val'] + 1,
					'h_val': h_values[child_loc],
					'parent': curr,
					'timestep': curr['timestep'] + 1}
			if (child['loc'], child['timestep']) in closed_list:
				existing_node = closed_list[(child['loc'], child['timestep'])]
				if compare_nodes(child, existing_node):
					closed_list[(child['loc'], child['timestep'])] = child
					push_node(open_list, child)
			else:
				closed_list[(child['loc'], child['timestep'])] = child
				push_node(open_list, child)
		path_length = path_length + 1

	return None  # Failed to find solutions

