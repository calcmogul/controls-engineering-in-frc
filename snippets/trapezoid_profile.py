"""Function for generating a trapezoid profile."""

import math


def generate_trapezoid_profile(max_v, time_to_max_v, dt, goal):
    """
    Creates a trapezoid profile with the given constraints.

    Parameter ``max_v``:
        Maximum velocity of profile.
    Parameter ``time_to_max_v``:
        Time from rest to maximum velocity.
    Parameter ``dt``:
        Timestep.
    Parameter ``goal``:
        Final position when the profile is at rest.
    Returns:
        t_rec -- List of timestamps.
        x_rec -- List of positions at each timestep.
        v_rec -- List of velocities at each timestep.
        a_rec -- List of accelerations at each timestep.
    """
    t_rec = [0.0]
    x_rec = [0.0]
    v_rec = [0.0]
    a_rec = [0.0]

    a = max_v / time_to_max_v
    time_at_max_v = goal / max_v - time_to_max_v

    # If profile is short
    if max_v * time_to_max_v > goal:
        time_to_max_v = math.sqrt(goal / a)
        time_from_max_v = time_to_max_v
        time_total = 2.0 * time_to_max_v
        profile_max_v = a * time_to_max_v
    else:
        time_from_max_v = time_to_max_v + time_at_max_v
        time_total = time_from_max_v + time_to_max_v
        profile_max_v = max_v

    while t_rec[-1] < time_total:
        t = t_rec[-1] + dt
        t_rec.append(t)
        if t < time_to_max_v:
            # Accelerate up
            a_rec.append(a)
            v_rec.append(a * t)
        elif t < time_from_max_v:
            # Maintain max velocity
            a_rec.append(0.0)
            v_rec.append(profile_max_v)
        elif t < time_total:
            # Accelerate down
            decel_time = t - time_from_max_v
            a_rec.append(-a)
            v_rec.append(profile_max_v - a * decel_time)
        else:
            a_rec.append(0.0)
            v_rec.append(0.0)
        x_rec.append(x_rec[-1] + v_rec[-1] * dt)
    return t_rec, x_rec, v_rec, a_rec
