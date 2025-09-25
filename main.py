import heapq
import os

def load_dynamic_map(file_path):
    width = height = 0
    terrain_costs = []
    static_obstacles = set()
    dynamic_obstacles = {}

    with open(file_path, 'r') as f:
        lines = f.readlines()

    mode = None
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        if line.startswith("width:"):
            width = int(line.split(":")[1].strip())
        elif line.startswith("height:"):
            height = int(line.split(":")[1].strip())
        elif line == "terrain:":
            mode = "terrain"
        elif line.startswith("static:"):
            mode = "static"
            static_str = line.split(":",1)[1].strip()
            # Robust parsing static obstacles
            if static_str:
                coords = static_str.split(",")
                for c in coords:
                    c = c.strip().strip("()")
                    if not c:
                        continue
                    parts = c.split(",")
                    if len(parts) != 2:
                        continue
                    x, y = map(int, parts)
                    static_obstacles.add((x, y))
        elif line == "dynamic:":
            mode = "dynamic"
        else:
            if mode == "terrain":
                row = list(map(int, line.split()))
                terrain_costs.append(row)
            elif mode == "dynamic":
                if ":" in line:
                    t_str, positions_str = line.split(":",1)
                    timestep = int(t_str.strip())
                    positions = positions_str.strip().split()
                    pos_set = set()
                    for p in positions:
                        p = p.strip().strip("()")
                        if p:
                            x,y = map(int, p.split(","))
                            pos_set.add((x,y))
                    dynamic_obstacles[timestep] = pos_set

    return width, height, terrain_costs, static_obstacles, dynamic_obstacles

class GridEnvironment:
    def __init__(self, width, height, terrain_costs, static_obstacles, dynamic_obstacles=None):
        self.width = width
        self.height = height
        self.terrain_costs = terrain_costs  # 2D list of integers ≥1 representing move cost
        self.static_obstacles = static_obstacles  # set of (x,y) tuples
        self.dynamic_obstacles = dynamic_obstacles if dynamic_obstacles else {}  # {timestep: set of (x,y)}

    def in_bounds(self, pos):
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, pos, timestep=None):
        if pos in self.static_obstacles:
            return False
        if timestep is not None and timestep in self.dynamic_obstacles:
            if pos in self.dynamic_obstacles[timestep]:
                return False
        return True

    def neighbors(self, pos, timestep=None):
        x, y = pos
        candidates = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        results = []
        for c in candidates:
            if self.in_bounds(c) and self.passable(c, timestep):
                results.append(c)
        return results

class Agent:
    def __init__(self, environment, start, goal):
        self.env = environment
        self.start = start
        self.goal = goal
        self.path = []

    def move(self, pos):
        pass  # Placeholder for movement logic

def uniform_cost_search(env, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        curr_cost, current = heapq.heappop(frontier)
        if current == goal:
            break

        for next_node in env.neighbors(current):
            new_cost = cost_so_far[current] + env.terrain_costs[next_node[1]][next_node[0]]
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                heapq.heappush(frontier, (new_cost, next_node))
                came_from[next_node] = current

    if goal not in came_from:
        return None, float('inf')  # No path found

    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from[node]
    path.append(start)
    path.reverse()

    return path, cost_so_far[goal]

def simulate_agent(env, agent, max_timesteps=50):
    timestep = 0
    planned_path, cost = uniform_cost_search(env, agent.start, agent.goal)

    if not planned_path:
        print("No initial path found.")
        return

    print(f"Initial path cost: {cost}")

    while timestep < max_timesteps and timestep < len(planned_path):
        current_pos = planned_path[timestep]
        print(f"Timestep {timestep}: Agent at {current_pos}")

        if timestep + 1 < len(planned_path):
            next_pos = planned_path[timestep + 1]
            if not env.passable(next_pos, timestep + 1):
                print(f"Dynamic obstacle detected at {next_pos} on timestep {timestep + 1}. Replanning!")
                planned_path, cost = uniform_cost_search(env, current_pos, agent.goal)
                if not planned_path:
                    print("No path found after replanning. Stopping simulation.")
                    return
                print(f"New path planned with cost: {cost}")
                timestep = 0
                continue

        timestep += 1
    print("Simulation ended or max timestep reached.")

if __name__ == "__main__":
    print("Running from directory:", os.getcwd())
    map_file = r"C:\Users\DELL\Desktop\autonomous_delivery_agent\maps\dynamic_map.txt"

    width, height, terrain_costs, static_obs, dynamic_obs = load_dynamic_map(map_file)

    env = GridEnvironment(width, height, terrain_costs, static_obs, dynamic_obs)
    agent = Agent(env, (0,0), (9,9))

    print("Dynamic environment loaded.")
    print(f"Static obstacles: {static_obs}")
    print(f"Dynamic obstacles schedule: {dynamic_obs}")

    simulate_agent(env, agent)

