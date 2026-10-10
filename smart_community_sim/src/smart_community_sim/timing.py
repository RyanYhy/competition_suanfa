"""Freshness in simulation time, with a separate wall-clock liveness bound."""
import math


def fresh_sample(now_sim, received_sim, now_wall, received_wall,
                 sim_timeout=.35, wall_timeout=2.0):
    if received_sim is None or received_wall is None:
        return False
    values=(now_sim,received_sim,now_wall,received_wall)
    return (all(math.isfinite(v) for v in values) and
            0 <= now_sim-received_sim <= sim_timeout and
            0 <= now_wall-received_wall <= wall_timeout)


def limit_velocity(vx, vy, wz, max_translation=.18, max_rotation=.5):
    """Common actuator envelope for both planners, including wheel saturation."""
    if not all(math.isfinite(v) for v in (vx,vy,wz)):
        return 0.,0.,0.
    ratio=max(1.,math.hypot(vx,vy)/max_translation,abs(wz)/max_rotation)
    vx,vy,wz=vx/ratio,vy/ratio,wz/ratio
    peak=max(abs(vx-vy-.27*wz),abs(vx+vy+.27*wz),abs(vx+vy-.27*wz),abs(vx-vy+.27*wz))
    ratio=max(1.,peak/(.054*8.))
    return vx/ratio,vy/ratio,wz/ratio
