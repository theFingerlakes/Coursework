import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, \
    get_location, get_sum_of_cost
from mdd import construct_MDD_for_agent, updateMDD, construct_MDD
from mvc import compute_EWMVC
from heuristics import compute_CG, compute_DG, construct_dependency_graph



def detect_collision(path1, path2):
    timestep = max(len(path1), len(path2))
    for t in range(timestep):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2:
            return ([loc1], t)
        if t < timestep - 1:
            loc1_next = get_location(path1, t+1)
            loc2_next = get_location(path2, t+1)
            if loc1 == loc2_next and loc2 == loc1_next:
                return ([loc1, loc2], t+1)
    return None


def detect_collisions(paths):
    collisions = []
    num_of_agents = len(paths)
    for i in range(num_of_agents - 1):
        for j in range(i + 1, num_of_agents):
            collision_t = detect_collision(paths[i], paths[j])
            if collision_t is not None:
                collision = {'a1': i, 'a2': j,
                             'loc': collision_t[0], 'timestep': collision_t[1]}
                collisions.append(collision)
    return collisions


def standard_splitting(collision):
    constraints = []
    loc = collision['loc']
    timestep = collision['timestep']
    a1 = collision['a1']
    a2 = collision['a2']
    if len(loc) == 1:
        constraints.append(
            {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append(
            {'agent': a2, 'loc': loc, 'timestep': timestep, 'positive': False})
        return constraints
    if len(loc) == 2:
        reverse_loc = loc.copy()
        reverse_loc.reverse()
        constraints.append(
            {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        constraints.append({'agent': a2, 'loc': reverse_loc,
                            'timestep': timestep, 'positive': False})
        return constraints


def disjoint_splitting(collision):

    constraints = []
    loc = collision['loc']
    timestep = collision['timestep']
    a1 = collision['a1']
    a2 = collision['a2']
    lucky_number = random.randint(0, 1)
    if len(loc) == 1:
        if lucky_number == 0:
            constraints.append(
                {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': True})
            constraints.append(
                {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        else:
            constraints.append(
                {'agent': a2, 'loc': loc, 'timestep': timestep, 'positive': True})
            constraints.append(
                {'agent': a2, 'loc': loc, 'timestep': timestep, 'positive': False})
        return constraints
    if len(loc) == 2:
        reverse_loc = loc.copy()
        reverse_loc.reverse()
        if lucky_number == 0:
            constraints.append(
                {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': True})
            constraints.append(
                {'agent': a1, 'loc': loc, 'timestep': timestep, 'positive': False})
        else:
            constraints.append(
                {'agent': a2, 'loc': reverse_loc, 'timestep': timestep, 'positive': True})
            constraints.append(
                {'agent': a2, 'loc': reverse_loc, 'timestep': timestep, 'positive': False})
        return constraints


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def compute_WDG(my_map, MDD, paths, constraints, num_of_agents, starts, goals):
    dependency_graph = construct_dependency_graph(
        num_of_agents, MDD, starts, goals)
    weighted_graph = compute_weights(
        my_map, paths, constraints, num_of_agents, dependency_graph, starts, goals)
    h_value = compute_EWMVC(weighted_graph)
    return h_value



def compute_weights(my_map, paths, constraints, num_of_agents, dependency_graph, starts, goals):
    G = dependency_graph.copy()
    for i in range(num_of_agents - 1):
        for j in range(i, num_of_agents):
            if (i, j) in G.edges:
                constraints_ij = [constraint.copy(
                ) for constraint in constraints if constraint['agent'] == i or constraint['agent'] == j]
                for constraint in constraints_ij:
                    if constraint['agent'] == i:
                        constraint['agent'] = 0
                    elif constraint['agent'] == j:
                        constraint['agent'] = 1
                starts_2 = [starts[i], starts[j]]
                goals_2 = [goals[i], goals[j]]
                cbs = CBSSolver(my_map, starts_2, goals_2)
                cost, root_paths, root_constraints = cbs.find_solution(
                    disjoint=False, heuristic='None', weight=True, constraints=constraints_ij)
                weight = cost - len(paths[i]) - len(paths[j]) + 2
                G.add_edge(i, j, weight=weight)
    return G


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristic = 'None'
        

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.construct_MDD = 0
        self.update_MDD = 0
        self.open_list = []

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(
            node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, heuristic='None', weight=False, constraints=[]):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
        self.heuristic = heuristic
        self.start_time = timer.time()

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'MDD': []}
        root['constraints'] = constraints.copy()
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])

        if heuristic != 'None':
            start_construct = timer.time()
            MDD = construct_MDD(self.my_map, self.num_of_agents, self.starts,
                                self.goals, self.heuristics, root['paths'], [])
            self.construct_MDD += timer.time() - start_construct
            h = 0
            if heuristic == 'CG':
                h_value = compute_CG(MDD, self.num_of_agents, self.starts)
                
            elif heuristic == 'DG':
                h = compute_DG(MDD, self.num_of_agents, self.starts, self.goals)
                

            elif heuristic == 'WDG':
                h = compute_WDG(self.my_map, MDD, root['paths'], root['constraints'], self.num_of_agents, self.starts, self.goals)
                

            root['MDD'] = MDD

            MDD_all = []
            for i in range(self.num_of_agents):
                mdd_i = {}
                mdd_i[len(root['paths'][i])-1] = MDD[i].copy()
                MDD_all.append(mdd_i)

        self.push_node(root)

        while len(self.open_list) > 0:
            P = self.pop_node()
            if len(P['collisions']) == 0:
                if weight:
                    cost = get_sum_of_cost(P['paths'])
                    return cost, P['paths'], root['constraints']
                self.print_results(P)
                return P['paths']
            collision = P['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            for constraint in constraints:
                isAdd = True
                Q = {}
                Q['constraints'] = P['constraints'] + [constraint]
                Q['paths'] = [path.copy() for path in P['paths']]
                Q['MDD'] = [MDD.copy() for MDD in P['MDD']]
                if constraint['positive'] == False:
                    a = constraint['agent']
                    path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                                  a, Q['constraints'])
                    if path is not None:
                        Q['paths'][a] = path.copy()
                        if heuristic != 'None':
                            if len(P['paths'][a]) < len(path):
                                mdd_temp = 0
                                if (len(path) - 1) in MDD_all[a]:

                                    mdd_temp = MDD_all[a][len(path)-1].copy()
                                else:
                                    start_construct = timer.time()
                                    mdd_temp = construct_MDD_for_agent(
                                        self.my_map, a, self.starts[a], self.goals[a], self.heuristics[a], len(path) - 1, [])
                                    self.construct_MDD += timer.time() - start_construct
                                    MDD_all[a][len(path)-1] = mdd_temp.copy()
                                Q['MDD'][a] = mdd_temp.copy()
                            start_update = timer.time()
                            Q['MDD'][a] = updateMDD(
                                Q['MDD'][a], a, self.starts[a], self.goals[a], len(path) - 1, Q['constraints'])
                            self.update_MDD += timer.time() - start_update
                    else:
                        isAdd = False

                if isAdd:
                    Q['collisions'] = detect_collisions(Q['paths'])
                    h_value = 0
                    if heuristic == 'CG':
                        h_value = compute_CG(Q['MDD'], self.num_of_agents, self.starts)
                        
                    elif heuristic == 'DG':
                        h_value = compute_DG(Q['MDD'], self.num_of_agents, self.starts, self.goals)
                        
                    elif heuristic == 'WDG':
                        h_value = compute_WDG(self.my_map, Q['MDD'], Q['paths'], Q['constraints'], self.num_of_agents, self.starts, self.goals)
                        

                    Q['cost'] = get_sum_of_cost(Q['paths']) + h_value

                    self.push_node(Q)

        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("Heuristic:    {}".format(self.heuristic))
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Run time (s):    {:.2f}".format(
            CPU_time-self.construct_MDD - self.update_MDD))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
