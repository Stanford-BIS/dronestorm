"""Utilities for controllers"""

def find_min_angle(angle_degrees):
    """Finds the minimum angle to 0 degrees"""
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360
    return angle_degrees 
