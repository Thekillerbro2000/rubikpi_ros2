"""Turn a waypoints file into a list of moves (used by waypoint_follower.py).

Waypoint file: one "x, y, theta" per line (metres, metres, radians); the first line is the
start pose. For each waypoint: turn to face it, drive to it, turn to its heading.
Zero turns / zero drives are skipped. Headings are rounded to whole degrees
(1.57 rad -> 90, -0.78 rad -> -45).

Can also be run on its own to print the moves:
    python waypoints_to_moves.py                  # uses ../config/waypoints.txt
    python waypoints_to_moves.py other_file.txt
"""
import math
import os
import sys
from pprint import pprint


def read_waypoints(path):
    """Return [(x, y, heading_deg), ...]."""
    waypoints = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            x, y, theta = (float(v) for v in line.split(','))
            waypoints.append((x, y, round(math.degrees(theta))))
    return waypoints


def turn_amount(from_deg, to_deg):
    """Signed turn in degrees, -180..180 (+ = left / counter-clockwise, - = right)."""
    return (to_deg - from_deg + 180) % 360 - 180


def waypoints_to_moves(waypoints):
    """List of ('drive', metres, wp) and ('turn', from_heading_deg, to_heading_deg, wp)."""
    moves = []
    x, y, heading = waypoints[0]
    for wp, (goal_x, goal_y, goal_heading) in enumerate(waypoints[1:], start=1):
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        if distance > 0:
            travel = round(math.degrees(math.atan2(dy, dx)))
            if turn_amount(heading, travel) != 0:
                moves.append(('turn', heading, travel, wp))
            moves.append(('drive', distance, wp))
            heading = travel
        if turn_amount(heading, goal_heading) != 0:
            moves.append(('turn', heading, goal_heading, wp))
        x, y, heading = goal_x, goal_y, goal_heading
    return moves


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'config', 'waypoints.txt')
    moves = waypoints_to_moves(read_waypoints(path))
    pprint(moves)
