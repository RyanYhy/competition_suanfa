"""Compare actual body motion with wheel odometry, using relative start/end poses."""
import math


def displacement(start, end):
    if not all(math.isfinite(v) for v in (*start,*end)):
        return (math.nan,math.nan,math.nan)
    dx,dy=end[0]-start[0],end[1]-start[1]
    c,s=math.cos(start[2]),math.sin(start[2])
    turn=math.atan2(math.sin(end[2]-start[2]),math.cos(end[2]-start[2]))
    return c*dx+s*dy,-s*dx+c*dy,turn


def compare_motion(true_start, true_end, odom_start, odom_end):
    actual=displacement(true_start,true_end);encoded=displacement(odom_start,odom_end)
    error=math.hypot(encoded[0]-actual[0],encoded[1]-actual[1])
    yaw_error=abs(math.atan2(math.sin(encoded[2]-actual[2]),math.cos(encoded[2]-actual[2])))
    finite=all(math.isfinite(v) for v in (*actual,*encoded,error,yaw_error))
    return dict(true_delta=dict(zip(('forward_m','lateral_m','yaw_rad'),actual)),
                odom_delta=dict(zip(('forward_m','lateral_m','yaw_rad'),encoded)),
                displacement_error_m=error,yaw_error_rad=yaw_error,
                odometry_consistent=finite and error<=.05 and yaw_error<=.15)


def stationary_check(true_start,true_end,odom_start,odom_end):
    comparison=compare_motion(true_start,true_end,odom_start,odom_end)
    actual=comparison['true_delta'];encoded=comparison['odom_delta']
    true_distance=math.hypot(actual['forward_m'],actual['lateral_m'])
    odom_distance=math.hypot(encoded['forward_m'],encoded['lateral_m'])
    return dict(test='stationary_hold',true_drift_m=true_distance,odom_drift_m=odom_distance,
                true_turn_rad=actual['yaw_rad'],odom_turn_rad=encoded['yaw_rad'],
                **comparison,**{'pass':comparison['odometry_consistent'] and true_distance<.01 and odom_distance<.015
                       and abs(actual['yaw_rad'])<.03 and abs(encoded['yaw_rad'])<.05})
