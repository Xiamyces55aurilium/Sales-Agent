import heapq
import os
import time

# Set this to your full absolute path for your maps folder:
MAP_FOLDER = r"C:\Users\DELL\Desktop\autonomous_delivery_agent\maps"

LOG_FILE = r"C:\Users\DELL\Desktop\autonomous_delivery_agent\logs\replanning_log.txt"

def log_event(message):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    with open(LOG_FILE, 'a') as f:
        f.write(f"[{timestamp}] {message}\n")

def load_map(file_path):
    width = height = 0
    terrain = []
    static_obs = set()
    dynamic_obs = dict()
    mode = None
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('width:'):
                width = int(line.split(':')[1])
            elif line.startswith('height:'):
                height = int(line.split(':')[1])
            elif line == 'terrain:':
                mode = 'terrain'
            elif line.startswith('static:'):
                mode = 'static'
                static_str = line.split(':',1)[1].strip()
                if static_str:
                    parts = static_str.split(',')
                    for p in parts:
                        p = p.strip().strip('()')
                        if p:
                            parts_static = p.split(',')
                            if len(parts_static) != 2:
                                print(f"Warning: Skipping malformed static coordinate: {p}")
                                continue
                            try:
                                x_static, y_static = map(int, parts_static)
                                static_obs.add((x_static, y_static))
                            except ValueError:
                                print(f"Warning: Skipping invalid static coordinate: {p}")
            elif line == 'dynamic:':
                mode = 'dynamic'
            else:
                if mode == 'terrain':
                    terrain.append(list(map(int, line.split())))
                elif mode == 'dynamic':
                    if ':' in line:
                        ts_str, positions = line.split(':', 1)
                        ts = int(ts_str.strip())
                        pos_set = set()
                        for pos in positions.strip().split():
                            pos = pos.strip().strip('()')
                            if pos:  # handle empty strings
                                parts_dyn = pos.split(',')
                                if len(parts_dyn) != 2:
                                    print(f"Warning: Skipping malformed dynamic obstacle coordinate: {pos}")
                                    continue
                                try:
                                    x_dyn, y_dyn = map(int, parts_dyn)
                                    pos_set.add((x_dyn, y_dyn))
                                except ValueError:
                                    print(f"Warning: Skipping invalid dynamic obstacle coordinate: {pos}")
                        dynamic_obs[ts] = pos_set
    return width, height, terrain, static_obs, dynamic_obs

class GridEnvironment:
    def __init__(self, width, height, terrain, static_obstacles, dynamic_obstacles=None):
        self.width = width
        self.height = height
        self.terrain = terrain
        self.static_obstacles = static_obstacles
        self.dynamic_obstacles = dynamic_obstacles if dynamic_obstacles else {}

    def in_bounds(self, pos):
        x,y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, pos, timestep=None):
        if pos in self.static_obstacles:
            return False
        if timestep is not None:
            if timestep in self.dynamic_obstacles:
                if pos in self.dynamic_obstacles[timestep]:
                    return False
        return True

    def neighbors(self, pos, timestep=None):
        x,y = pos
        nbrs = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        return [p for p in nbrs if self.in_bounds(p) and self.passable(p,timestep)]


class Agent:
    def __init__(self, env, start, goal):
        self.env = env
        self.start = start
        self.goal = goal
        self.path = []


def uniform_cost_search(env, start, goal):
    frontier = []
    heapq.heappush(frontier,(0,start))
    came_from = {start:None}
    cost_so_far = {start:0}
    nodes_expanded = 0
    while frontier:
        cost,current = heapq.heappop(frontier)
        nodes_expanded +=1
        if current==goal:
            break
        for next_node in env.neighbors(current):
            new_cost = cost_so_far[current] + env.terrain[next_node[1]][next_node[0]]
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                heapq.heappush(frontier,(new_cost,next_node))
                came_from[next_node] = current
    if goal not in came_from:
        return None,float('inf'),nodes_expanded
    path = []
    cur = goal
    while cur != start:
        path.append(cur)
        cur = came_from[cur]
    path.append(start)
    path.reverse()
    return path,cost_so_far[goal],nodes_expanded


def hill_climbing_replan(env, current_pos, goal, max_restarts=5):
    best_path, best_cost, _ = uniform_cost_search(env, current_pos, goal)
    if best_path is None:
        return None,float('inf')
    for _ in range(max_restarts):
        path, cost, _ = uniform_cost_search(env, current_pos, goal)
        if path and cost < best_cost:
            best_path,best_cost=path,cost
    return best_path,best_cost


def simulate_agent(env, agent, max_timesteps=100):
    timestep=0
    planned_path, cost, nodes_expanded = uniform_cost_search(env, agent.start, agent.goal)
    if not planned_path:
        print("No initial path found")
        return
    print(f"Initial path found with cost {cost}, nodes expanded {nodes_expanded}")
    agent.path = planned_path
    while timestep < max_timesteps and timestep < len(agent.path):
        pos = agent.path[timestep]
        print(f"Timestep {timestep}: Agent at {pos}")
        if timestep+1 < len(agent.path):
            next_pos = agent.path[timestep+1]
            if not env.passable(next_pos,timestep+1):
                print(f"Dynamic obstacle detected at {next_pos} timestep {timestep+1}, replanning...")
                log_event(f"Dynamic obstacle at {next_pos} on timestep {timestep+1}. Replanning.")
                new_path,new_cost = hill_climbing_replan(env,pos,agent.goal)
                if not new_path:
                    print("No path found after replanning. Simulation ends.")
                    return
                print(f"New path cost {new_cost}")
                log_event(f"New path cost {new_cost} after replanning.")
                agent.path = new_path
                timestep=0
                continue
        timestep+=1
    print("Simulation finished.")


def main():
    if not os.path.exists(os.path.dirname(LOG_FILE)):
        os.makedirs(os.path.dirname(LOG_FILE))
    map_file = os.path.join(MAP_FOLDER,"dynamic_map.txt")
    if not os.path.exists(map_file):
        print(f"Map file not found: {map_file}")
        return
    width,height,terrain,static_obs,dynamic_obs = load_map(map_file)
    env = GridEnvironment(width,height,terrain,static_obs,dynamic_obs)
    agent = Agent(env,(0,0),(width-1,height-1))
    print(f"Loaded map from {map_file}")
    simulate_agent(env,agent)


if __name__ == "__main__":
    main()


