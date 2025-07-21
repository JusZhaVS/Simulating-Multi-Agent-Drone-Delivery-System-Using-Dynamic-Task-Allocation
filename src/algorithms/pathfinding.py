import heapq
from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass, field

@dataclass
class Node:
    """
    Node class for A* pathfinding algorithm.
    Demonstrates understanding of data structures and complexity analysis.
    """
    position: Tuple[int, int]
    g_cost: float = float('inf')  # Cost from start to this node
    h_cost: float = 0  # Heuristic cost from this node to goal
    f_cost: float = field(init=False)  # Total cost (g + h)
    parent: Optional['Node'] = None
    
    def __post_init__(self):
        self.f_cost = self.g_cost + self.h_cost
        
    def __lt__(self, other):
        """Enable priority queue ordering by f_cost."""
        if self.f_cost != other.f_cost:
            return self.f_cost < other.f_cost
        return self.h_cost < other.h_cost

class AStarPathfinder:
    """
    A* pathfinding algorithm implementation for robot navigation.
    
    Time Complexity: O(b^d) where b is branching factor and d is depth
    Space Complexity: O(b^d) for storing nodes in open/closed sets
    
    This demonstrates:
    - Algorithm implementation and complexity analysis
    - Data structures (priority queue, sets, dictionaries)
    - Graph search algorithms
    """
    
    def __init__(self, city_grid):
        self.city_grid = city_grid
        
    def manhattan_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """
        Manhattan distance heuristic for A* algorithm.
        Admissible and consistent for grid-based movement.
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
        
    def euclidean_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """
        Euclidean distance heuristic (alternative).
        More accurate but potentially inadmissible for grid movement.
        """
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
        
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int], 
                  heuristic: str = "manhattan") -> Optional[List[Tuple[int, int]]]:
        """
        Find optimal path from start to goal using A* algorithm.
        
        Args:
            start: Starting position (x, y)
            goal: Goal position (x, y)
            heuristic: "manhattan" or "euclidean"
            
        Returns:
            List of positions representing the path, or None if no path exists
            
        Time Complexity: O(b^d) - exponential in worst case
        Space Complexity: O(b^d) - stores all explored nodes
        """
        if not self.city_grid.is_valid_location(start) or not self.city_grid.is_valid_location(goal):
            return None
            
        if start == goal:
            return [start]
            
        # Choose heuristic function
        heuristic_func = self.manhattan_distance if heuristic == "manhattan" else self.euclidean_distance
        
        # Initialize data structures
        open_set = []  # Priority queue (min-heap)
        closed_set: Set[Tuple[int, int]] = set()
        nodes: Dict[Tuple[int, int], Node] = {}
        
        # Create start node
        start_node = Node(start, g_cost=0, h_cost=heuristic_func(start, goal))
        nodes[start] = start_node
        heapq.heappush(open_set, start_node)
        
        while open_set:
            # Get node with lowest f_cost
            current_node = heapq.heappop(open_set)
            current_pos = current_node.position
            
            # Goal reached
            if current_pos == goal:
                return self._reconstruct_path(current_node)
                
            closed_set.add(current_pos)
            
            # Explore neighbors
            for neighbor_pos in self.city_grid.get_neighbors(current_pos):
                if neighbor_pos in closed_set:
                    continue
                    
                # Calculate tentative g_cost
                tentative_g = current_node.g_cost + 1  # Assuming unit cost per move
                
                # Create neighbor node if it doesn't exist
                if neighbor_pos not in nodes:
                    neighbor_node = Node(
                        neighbor_pos,
                        g_cost=float('inf'),
                        h_cost=heuristic_func(neighbor_pos, goal)
                    )
                    nodes[neighbor_pos] = neighbor_node
                else:
                    neighbor_node = nodes[neighbor_pos]
                
                # Update neighbor if we found a better path
                if tentative_g < neighbor_node.g_cost:
                    neighbor_node.parent = current_node
                    neighbor_node.g_cost = tentative_g
                    neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost
                    
                    # Add to open set if not already there
                    if neighbor_node not in open_set:
                        heapq.heappush(open_set, neighbor_node)
                        
        return None  # No path found
        
    def _reconstruct_path(self, goal_node: Node) -> List[Tuple[int, int]]:
        """
        Reconstruct path from goal node to start by following parent pointers.
        """
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
            
        return path[::-1]  # Reverse to get start-to-goal path
        
    def find_path_with_waypoints(self, start: Tuple[int, int], 
                                waypoints: List[Tuple[int, int]]) -> Optional[List[Tuple[int, int]]]:
        """
        Find path through multiple waypoints (useful for pickup->dropoff->charging station).
        
        Returns complete path visiting all waypoints in order.
        """
        if not waypoints:
            return [start]
            
        complete_path = [start]
        current_pos = start
        
        for waypoint in waypoints:
            segment_path = self.find_path(current_pos, waypoint)
            if segment_path is None:
                return None  # No path to this waypoint
                
            # Add segment path (excluding start to avoid duplicates)
            complete_path.extend(segment_path[1:])
            current_pos = waypoint
            
        return complete_path
        
    def calculate_path_cost(self, path: List[Tuple[int, int]]) -> float:
        """
        Calculate total cost of a path.
        """
        if len(path) < 2:
            return 0.0
            
        total_cost = 0.0
        for i in range(len(path) - 1):
            total_cost += 1  # Unit cost per move
            
        return total_cost
        
    def optimize_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Simple path optimization using line-of-sight checks.
        Removes unnecessary waypoints when direct path is available.
        """
        if len(path) <= 2:
            return path
            
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Look ahead to find furthest reachable point
            j = len(path) - 1
            while j > i + 1:
                if self._has_line_of_sight(path[i], path[j]):
                    optimized.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                # No line of sight found, move to next point
                optimized.append(path[i + 1])
                i += 1
                
        return optimized
        
    def _has_line_of_sight(self, start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """
        Check if there's a direct line of sight between two points.
        Uses Bresenham's line algorithm for efficiency.
        """
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if not self.city_grid.is_valid_location((x, y)):
                return False
                
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return True