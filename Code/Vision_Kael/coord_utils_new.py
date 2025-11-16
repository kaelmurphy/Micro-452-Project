"""
Coordinate utilities for robot vision system.
Provides smoothing and transformation functions for ArUco marker-based navigation.
"""
from typing import Dict, Optional, Tuple, Any
import numpy as np

# Smoothing parameters
SMOOTHING_ALPHA: float = 0.4  # exponential moving average factor
_smoothing_data: Dict[str, Dict[str, Any]] = {}  # global state for smoothing


def smoothTuple(smooth_key: str, data_key: str, new_value: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """
    Apply exponential moving average smoothing to tuple values (x, y).
    
    Args:
        smooth_key: Unique identifier for the smoothing context
        data_key: Key for the specific data being smoothed
        new_value: New (x, y) value to incorporate
        
    Returns:
        Smoothed (x, y) tuple or None if input is invalid
    """
    if not new_value or len(new_value) < 2:
        return None
        
    if smooth_key not in _smoothing_data:
        _smoothing_data[smooth_key] = {}
        
    if data_key not in _smoothing_data[smooth_key]:
        _smoothing_data[smooth_key][data_key] = tuple(new_value[:2])
        return _smoothing_data[smooth_key][data_key]
        
    old = _smoothing_data[smooth_key][data_key]
    smoothed = (
        SMOOTHING_ALPHA * new_value[0] + (1 - SMOOTHING_ALPHA) * old[0],
        SMOOTHING_ALPHA * new_value[1] + (1 - SMOOTHING_ALPHA) * old[1]
    )
    _smoothing_data[smooth_key][data_key] = smoothed
    return smoothed


def smoothAngle(smooth_key: str, data_key: str, new_angle: Optional[float]) -> Optional[float]:
    """
    Apply exponential moving average smoothing to angular values with wraparound handling.
    
    Args:
        smooth_key: Unique identifier for the smoothing context
        data_key: Key for the specific data being smoothed
        new_angle: New angle in degrees
        
    Returns:
        Smoothed angle in degrees [-180, 180] or None if input is invalid
    """
    if new_angle is None:
        return None
        
    if smooth_key not in _smoothing_data:
        _smoothing_data[smooth_key] = {}
        
    if data_key not in _smoothing_data[smooth_key]:
        _smoothing_data[smooth_key][data_key] = new_angle
        return new_angle
        
    old = _smoothing_data[smooth_key][data_key]
    # Handle angle wraparound for proper averaging
    diff = ((new_angle - old + 180) % 360) - 180
    smoothed = old + SMOOTHING_ALPHA * diff
    smoothed = (smoothed + 180) % 360 - 180  # normalize to [-180, 180]
    _smoothing_data[smooth_key][data_key] = smoothed
    return smoothed


def asXy(value: Any) -> bool:
    """
    Validate that value is a tuple/list with 2 numeric elements.
    
    Args:
        value: Value to validate
        
    Returns:
        True if value is a valid (x, y) coordinate pair
    """
    return value is not None and len(value) >= 2 and value[0] is not None and value[1] is not None


def robotWorldPose(
    centers: Dict[int, Tuple[float, float]], 
    corners_map: Optional[Dict[int, np.ndarray]], 
    robot_id: int,
    axis_x: Optional[np.ndarray] = None, 
    axis_y: Optional[np.ndarray] = None
) -> Tuple[Optional[int], Optional[int], Optional[float]]:
    """
    Compute robot center (x,y) and orientation theta (radians) from marker corners.
    
    Args:
        centers: Dictionary mapping marker IDs to center coordinates
        corners_map: Dictionary mapping marker IDs to corner arrays
        robot_id: ID of the robot marker
        axis_x: X-axis direction of coordinate system (optional)
        axis_y: Y-axis direction of coordinate system (optional)
        
    Returns:
        Tuple of (center_x, center_y, theta_radians) or (None, None, None) if not found
    """
    if robot_id not in centers:
        return (None, None, None)
        
    # Extract center coordinates
    try:
        cx, cy = int(centers[robot_id][0]), int(centers[robot_id][1])
    except (KeyError, IndexError, ValueError, TypeError):
        return (None, None, None)
    
    # Compute orientation from marker corners if available
    if corners_map and robot_id in corners_map:
        arr = corners_map[robot_id].astype(float)
        # Midpoint of top edge (between top-left and top-right corners)
        top_mid = (arr[0] + arr[1]) / 2.0
        # Vector from center to top midpoint (robot's forward direction in pixel coordinates)
        robot_forward_px = np.array([top_mid[0] - cx, top_mid[1] - cy])
        
        # If we have coordinate axes, calculate angle relative to the coordinate system
        if axis_x is not None and axis_y is not None:
            coord_x_px = np.array(axis_x)  # X-axis direction in world coordinates
            
            # Calculate robot angle in world coordinates relative to X-axis
            if np.linalg.norm(robot_forward_px) > 0:
                robot_forward_norm = robot_forward_px / np.linalg.norm(robot_forward_px)
                
                # Calculate angle between robot forward and coordinate X-axis
                # Since Y is inverted in pixels, we need to account for this
                robot_forward_world = np.array([robot_forward_norm[0], -robot_forward_norm[1]])
                
                # Angle relative to coordinate system X-axis
                theta = float(np.arctan2(
                    np.cross(coord_x_px, robot_forward_world), 
                    np.dot(coord_x_px, robot_forward_world)
                ))
                return (cx, cy, theta)
        
        # Fallback: calculate angle relative to pixel coordinate system
        theta = float(np.arctan2(top_mid[1] - cy, top_mid[0] - cx))
        return (cx, cy, theta)
        
    return (cx, cy, None)