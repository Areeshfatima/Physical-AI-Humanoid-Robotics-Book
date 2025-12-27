"""Navigation planner for the VLA system."""

from typing import List, Tuple, Optional
import numpy as np
from ..models.navigation_plan import NavigationPlan
from ..models.path_point import PathPoint
import uuid


class NavigationPlanner:
    """Class to handle path planning for robot navigation."""
    
    def __init__(self):
        """Initialize the navigation planner."""
        self.max_planning_attempts = 5
        self.min_distance_threshold = 0.1  # meters
        self.default_max_velocity = 0.5  # m/s
    
    def plan_path(self, 
                  start_position: Tuple[float, float, float], 
                  goal_position: Tuple[float, float, float],
                  map_data: Optional[dict] = None) -> Optional[NavigationPlan]:
        """
        Plan a path from start position to goal position.
        
        Args:
            start_position: Starting (x, y, z) coordinates
            goal_position: Target (x, y, z) coordinates
            map_data: Optional map data for advanced path planning
            
        Returns:
            NavigationPlan object or None if planning fails
        """
        try:
            # Calculate straight-line distance to goal
            dx = goal_position[0] - start_position[0]
            dy = goal_position[1] - start_position[1]
            dz = goal_position[2] - start_position[2]
            
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # If already at goal, return empty plan
            if distance < self.min_distance_threshold:
                return self._create_empty_navigation_plan(goal_position)
            
            # For simplicity in this implementation, we'll create a direct path
            # In a real system, this would involve more sophisticated path planning
            # considering obstacles, map constraints, etc.
            
            # Create path with intermediate waypoints
            path_points = self._create_direct_path(start_position, goal_position)
            
            # Calculate estimated duration based on distance and default velocity
            estimated_duration = distance / self.default_max_velocity
            
            # Create navigation plan
            nav_plan = NavigationPlan(
                id=str(uuid.uuid4()),
                destination_x=goal_position[0],
                destination_y=goal_position[1],
                destination_z=goal_position[2],
                path=path_points,
                estimated_duration=estimated_duration,
                status="ready"  # Ready for execution
            )
            
            return nav_plan
            
        except Exception as e:
            print(f"Error planning path: {str(e)}")
            return None
    
    def _create_direct_path(self, start: Tuple[float, float, float], 
                           goal: Tuple[float, float, float]) -> List[PathPoint]:
        """Create a direct path from start to goal with intermediate waypoints."""
        try:
            # For this basic implementation, we'll create a direct path
            # with a few intermediate points
            
            # Calculate vector from start to goal
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            dz = goal[2] - start[2]
            
            # Total distance
            total_distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Create waypoints at regular intervals (every 0.5 meters)
            waypoint_distance = min(0.5, total_distance)  # Don't create waypoints if very close
            num_waypoints = int(total_distance / waypoint_distance) if waypoint_distance > 0 else 0
            
            path_points = []
            
            # Add start point as first waypoint if not too close to goal
            if total_distance > waypoint_distance:
                start_point = PathPoint(
                    id=f"start_{uuid.uuid4()}",
                    x=start[0],
                    y=start[1],
                    z=start[2],
                    sequence_number=0
                )
                path_points.append(start_point)
            
            # Add intermediate waypoints
            for i in range(1, num_waypoints + 1):
                fraction = i / (num_waypoints + 1) if num_waypoints > 0 else 0.5
                
                waypoint_x = start[0] + dx * fraction
                waypoint_y = start[1] + dy * fraction
                waypoint_z = start[2] + dz * fraction
                
                waypoint = PathPoint(
                    id=f"wp_{i}_{uuid.uuid4()}",
                    x=waypoint_x,
                    y=waypoint_y,
                    z=waypoint_z,
                    sequence_number=i
                )
                path_points.append(waypoint)
            
            # Add goal as final waypoint
            goal_point = PathPoint(
                id=f"goal_{uuid.uuid4()}",
                x=goal[0],
                y=goal[1],
                z=goal[2],
                sequence_number=len(path_points)
            )
            path_points.append(goal_point)
            
            return path_points
            
        except Exception as e:
            print(f"Error creating direct path: {str(e)}")
            # Return a path with just the goal point as fallback
            return [PathPoint(
                id=f"goal_{uuid.uuid4()}",
                x=goal[0],
                y=goal[1],
                z=goal[2],
                sequence_number=0
            )]
    
    def _create_empty_navigation_plan(self, position: Tuple[float, float, float]) -> NavigationPlan:
        """Create an empty navigation plan when already at the destination."""
        return NavigationPlan(
            id=str(uuid.uuid4()),
            destination_x=position[0],
            destination_y=position[1],
            destination_z=position[2],
            path=[],
            estimated_duration=0.0,
            status="completed"  # Already at destination
        )
    
    def validate_path(self, path: List[PathPoint], map_data: Optional[dict] = None) -> bool:
        """Validate a planned path for collisions or other issues."""
        try:
            # In a real implementation, this would check for collisions with obstacles
            # For this basic implementation, we'll just verify the path is reasonable
            
            if not path:
                return True  # An empty path is valid if already at destination
            
            # Check that path points are properly sequenced
            for i in range(len(path) - 1):
                if path[i].sequence_number + 1 != path[i+1].sequence_number:
                    print("Path points not properly sequenced")
                    return False
            
            # If map data is provided, more sophisticated validation could occur here
            # For now, assume path is valid
            return True
            
        except Exception as e:
            print(f"Error validating path: {str(e)}")
            return False