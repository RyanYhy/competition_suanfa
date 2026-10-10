"""Numeric metrics shared by the live benchmark and offline tests."""
import math
import numpy as np


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def goal_accuracy(goals, expected_names):
    expected=set(expected_names)
    observed={g['name'] for g in goals}
    missing=sorted(expected-observed)
    passed=bool(expected) and not missing and all(
        math.isfinite(g['position_error_m']) and math.isfinite(g['heading_error_deg']) and
        g['position_error_m']<=.05 and g['heading_error_deg']<=5 for g in goals)
    return dict(accuracy_pass=passed,missing_goals=missing)


def summarize(rows):
    if not rows:return {'samples':0,'valid':False}
    def stats(values):
        a=np.asarray(values,dtype=float)
        return dict(rmse=float(np.sqrt(np.mean(a*a))),p95=float(np.percentile(np.abs(a),95)),maximum=float(np.max(np.abs(a))))
    position=[math.hypot(r['est_x']-r['true_x'],r['est_y']-r['true_y']) for r in rows]
    heading=[math.degrees(wrap(r['est_yaw']-r['true_yaw'])) for r in rows]
    odometry=[math.hypot(r['odom_x']-r['true_x'],r['odom_y']-r['true_y']) for r in rows]
    moving=[r for r in rows if r['phase']=='navigating']
    sim_span=rows[-1]['sim_time']-rows[0]['sim_time'];wall_span=rows[-1]['wall_time']-rows[0]['wall_time']
    return dict(samples=len(rows),valid=True,position_error_m=stats(position),heading_error_deg=stats(heading),
                odom_error_m=stats(odometry),sim_duration_s=sim_span,wall_duration_s=wall_span,
                real_time_factor=sim_span/wall_span if wall_span>0 else None,
                moving_mean_speed_mps=float(np.mean([r['true_speed'] for r in moving])) if moving else None,
                moving_planned_speed_mps=float(np.mean([r['planned_speed'] for r in moving])) if moving and all('planned_speed' in r for r in moving) else None,
                moving_odom_speed_mps=float(np.mean([r['odom_speed'] for r in moving])) if moving and all('odom_speed' in r for r in moving) else None,
                guard_counts={reason:sum(r.get('guard')==reason for r in moving) for reason in sorted({r['guard'] for r in moving if 'guard' in r})},
                moving_zero_command_fraction=float(np.mean([r['cmd_speed']<.005 and abs(r['cmd_yaw'])<.01 for r in moving])) if moving else None)
