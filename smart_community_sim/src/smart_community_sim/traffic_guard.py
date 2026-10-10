#!/usr/bin/env python3
"""Stop-line guard using localized footprint, measured velocity and braking allowance."""
import math


STOP_LINES = {
    'north_signal': {'axis': 0, 'line': .30, 'low': 1.50, 'high': 2.10},
    'south_signal': {'axis': 1, 'line': -.55, 'low': -.40, 'high': .20},
}


def blocked_signal(x, y, yaw, vx, vy, measured_vx, measured_vy, green):
    """Both crossings travel toward decreasing coordinate; already-entered traffic clears.

    This is a conservative command filter, not proof of physical stopping distance.
    It covers the real footprint plus localization and command-latency allowances.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    velocity = (c*vx-s*vy, s*vx+c*vy)
    measured = (c*measured_vx-s*measured_vy, s*measured_vx+c*measured_vy)
    half = (.19*abs(c)+.17*abs(s), .19*abs(s)+.17*abs(c))
    position = (x, y)
    for name, line in STOP_LINES.items():
        axis = line['axis']; across = 1-axis
        if position[across]+half[across] < line['low'] or position[across]-half[across] > line['high']:
            continue
        leading = position[axis]-half[axis]
        if leading < line['line']-.0125:
            continue  # Clear an intersection already entered; do not stop under the lamp.
        speed = max(0., -velocity[axis], -measured[axis])
        braking = speed*speed/(2*.30)+speed*.20+.05
        if speed > 0 and leading-line['line'] <= .0125+braking and not green(name):
            return name
    return None
