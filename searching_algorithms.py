from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot
import math 
import heapq
from collections import defaultdict

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    queue = deque([start])
    visited = {start}
    came_from = {}
    while queue: 
        current_node = queue.popleft()
        if current_node == end:
            while current_node in came_from:
                current_node = came_from[current_node]
                current_node.make_path()
                draw()
            end.make_end(), start.make_start()
            return True
        for neighbor in current_node.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current_node
                queue.append(neighbor)
                neighbor.make_open()
        draw()

        if current_node != start:
            current_node.make_closed()
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    stack = [start]
    visited = {start}
    came_from = {}
    while stack: 
        current_node = stack.pop()
        if current_node == end:
            while current_node in came_from:
                current_node = came_from[current_node]
                current_node.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current_node.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                came_from[neighbor] = current_node
                stack.append(neighbor)
                neighbor.make_open()
        draw()

        if current_node != start:
            current_node.make_closed()
    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    return abs(p2[0]-p1[0]) + abs(p2[1]-p1[1])

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Euclidean distance between p1 and p2.
    """
    return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]))


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))

    came_from = {}

    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0

    f_score = {spot: float("inf") for row in grid.grid for spot in row}
    f_score[start] = h_manhattan_distance(start.get_position(), end.get_position())

    lookup_set = {start}

    while not open_heap.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False
                
        current = open_heap.get()[2]
        lookup_set.remove(current)

        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            tentative_g = g_score[current] + 1

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h_manhattan_distance(neighbor.get_position(), end.get_position())

                if neighbor not in lookup_set:
                    count += 1
                    open_heap.put((f_score[neighbor], count, neighbor))
                    lookup_set.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False 

def dls(draw: callable, grid: Grid, start: Spot, end: Spot, limit: int) -> bool:
    stack = [(start, 0)]
    visited = {start}
    came_from = {}
    while stack: 
        current_node, depth = stack.pop()
        if current_node == end:
            while current_node in came_from:
                current_node = came_from[current_node]
                current_node.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        if depth < limit:
            for neighbor in current_node.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    visited.add(neighbor)
                    came_from[neighbor] = current_node
                    stack.append((neighbor, depth + 1))
                    neighbor.make_open()
            draw()

        if current_node != start:
            current_node.make_closed()
    return False

def ucs(draw: callable, grid: Grid, start: Spot, end: Spot):
    queue = []
    heapq.heappush(queue, (0, start))
    visited = {start : (None, 0)}
    while queue: 
        current_cost, current_node = heapq.heappop(queue)
        if current_node == end:
            current = end
            while current != start:
                parent, _ = visited[current]
                current = parent
                current.make_path()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current_node.neighbors:
            if neighbor.is_barrier():
                continue
            step_cost = 1
            new_cost = current_cost + step_cost
            if neighbor not in visited or new_cost < visited[neighbor][1]:
                visited[neighbor] = (current_node, new_cost)
                heapq.heappush(queue, (new_cost, neighbor))
                neighbor.make_open()
        draw()
        if current_node != start:
            current_node.make_closed()
    return None

def gbfs(draw: callable, grid: Grid, start: Spot, end: Spot, region_map: dict[Spot, Spot]):
    queue = []
    heapq.heappush(queue, (h_manhattan_distance(start.get_position(), end.get_position()), start))
    visited = set([start])
    came_from = {start: None}
    while queue:
        _, current = heapq.heappop(queue)
        if current == end:
            while current != start:
                current = came_from[current]
                current.make_path()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            row = neighbor.row
            col = neighbor.col
            if region_map.get(neighbor, 0) == 999:
                continue
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                heapq.heappush(queue, (h_manhattan_distance(neighbor.get_position(), end.get_position()), neighbor))
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return None

def iddfs(draw: callable, grid: Grid, start: Spot, end: Spot, max_depth: int) -> bool:
    for depth in range(max_depth + 1):
        if dls(draw, grid, start, end, depth):
            return True
    return False

def idastar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    threshold = h_manhattan_distance(start.get_position(), end.get_position())
    while True:
        stack = [(start, 0, {})]
        visited = {start}
        mini = math.inf
        while stack:
            current_node, g, came_from = stack.pop()
            f = g + h_manhattan_distance(current_node.get_position(), end.get_position())
            if f > threshold:
                mini = min(mini, f)
                continue
            if current_node == end:
                while current_node in came_from:
                    current_node = came_from[current_node]
                    current_node.make_path()
                    draw()
                end.make_end()
                start.make_start()
                return True
            for neighbor in current_node.neighbors:
                if neighbor.is_barrier():
                    continue
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from_copy = came_from.copy()
                    came_from_copy[neighbor] = current_node
                    neighbor.make_open()
                    stack.append((neighbor, g+1, came_from_copy))
            draw()
            if current_node != start and current_node != end:
                current_node.make_closed()
        if mini == math.inf:
            return False
        threshold = mini

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
# ▢ Uninformed Cost Search (UCS)
# ▢ Greedy Search
# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss