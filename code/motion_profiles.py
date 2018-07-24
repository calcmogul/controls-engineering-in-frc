#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def generate_trapezoid_profile(max_v, time_to_max_v, dt, goal):
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


def generate_s_curve_profile(max_v, max_a, time_to_max_a, dt, goal):
    t_rec = [0.0]
    x_rec = [0.0]
    v_rec = [0.0]
    a_rec = [0.0]

    j = max_a / time_to_max_a
    short_profile = max_v * (time_to_max_a + max_v / max_a) > goal

    if short_profile:
        profile_max_v = max_a * (math.sqrt(
            goal / max_a - 0.75 * time_to_max_a**2) - 0.5 * time_to_max_a)
    else:
        profile_max_v = max_v

    # Find times at critical points
    t2 = profile_max_v / max_a
    t3 = t2 + time_to_max_a
    if short_profile:
        t4 = t3
    else:
        t4 = goal / profile_max_v
    t5 = t4 + time_to_max_a
    t6 = t4 + t2
    t7 = t6 + time_to_max_a
    time_total = t7

    while t_rec[-1] < time_total:
        t = t_rec[-1] + dt
        t_rec.append(t)
        if t < time_to_max_a:
            # Ramp up acceleration
            a_rec.append(j * t)
            v_rec.append(0.5 * j * t**2)
        elif t < t2:
            # Increase speed at max acceleration
            a_rec.append(max_a)
            v_rec.append(max_a * (t - 0.5 * time_to_max_a))
        elif t < t3:
            # Ramp down acceleration
            a_rec.append(max_a - j * (t - t2))
            v_rec.append(max_a * (t - 0.5 * time_to_max_a) -
                         0.5 * j * (t - t2)**2)
        elif t < t4:
            # Maintain max velocity
            a_rec.append(0.0)
            v_rec.append(profile_max_v)
        elif t < t5:
            # Ramp down acceleration
            a_rec.append(-j * (t - t4))
            v_rec.append(profile_max_v - 0.5 * j * (t - t4)**2)
        elif t < t6:
            # Decrease speed at max acceleration
            a_rec.append(-max_a)
            v_rec.append(max_a * (t2 + t5 - t - 0.5 * time_to_max_a))
        elif t < t7:
            # Ramp up acceleration
            a_rec.append(-max_a + j * (t - t6))
            v_rec.append(max_a * (t2 + t5 - t - 0.5 * time_to_max_a) +
                         0.5 * j * (t - t6)**2)
        else:
            a_rec.append(0.0)
            v_rec.append(0.0)
        x_rec.append(x_rec[-1] + v_rec[-1] * dt)
    return t_rec, x_rec, v_rec, a_rec


def main():
    t, x, v, a = generate_trapezoid_profile(
        max_v=7.0, time_to_max_v=2.0, dt=0.05, goal=50.0)
    plt.figure(1)
    plt.subplot(311)
    plt.ylabel("Position (m)")
    plt.plot(t, x, label="Position")
    plt.subplot(312)
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, v, label="Velocity")
    plt.subplot(313)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration ($m/s^2$)")
    plt.plot(t, a, label="Acceleration")
    plt.savefig("trapezoid_profile.svg")

    t, x, v, a = generate_s_curve_profile(
        max_v=7.0, max_a=3.5, time_to_max_a=1.0, dt=0.05, goal=50.0)
    plt.figure(2)
    plt.subplot(311)
    plt.ylabel("Position (m)")
    plt.plot(t, x, label="Position")
    plt.subplot(312)
    plt.ylabel("Velocity (m/s)")
    plt.plot(t, v, label="Velocity")
    plt.subplot(313)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration ($m/s^2$)")
    plt.plot(t, a, label="Acceleration")
    plt.savefig("s_curve_profile.svg")


if __name__ == "__main__":
    main()
