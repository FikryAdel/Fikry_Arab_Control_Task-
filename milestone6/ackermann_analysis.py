"""
Milestone 6: Four-wheel Ackermann kinematics vs. the kinematic bicycle model.

Vehicle: wheelbase L = 1.25 m, track width W = 1.18 m.
Produces a table and a plot comparing steering angles and wheel speeds.
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

L = 1.25   # wheelbase (m)
W = 1.18   # track width (m)


def ackermann_angles(delta):
    """Returns (delta_inner, delta_outer) for a bicycle steering angle delta (rad > 0)."""
    if abs(delta) < 1e-9:
        return 0.0, 0.0
    R = L / math.tan(abs(delta))                  # turning radius at rear-axle center
    d_in = math.atan(L / (R - W / 2.0))
    d_out = math.atan(L / (R + W / 2.0))
    return math.copysign(d_in, delta), math.copysign(d_out, delta)


def rear_wheel_speeds(v, delta):
    """Returns (v_inner, v_outer) rear wheel speeds for body speed v and angle delta."""
    if abs(delta) < 1e-9:
        return v, v
    R = L / math.tan(abs(delta))
    omega = v / R
    return omega * (R - W / 2.0), omega * (R + W / 2.0)


def main():
    deltas_deg = np.linspace(1, 35, 69)
    d_in, d_out, err = [], [], []
    for dd in deltas_deg:
        di, do = ackermann_angles(math.radians(dd))
        d_in.append(math.degrees(di))
        d_out.append(math.degrees(do))
        err.append(math.degrees(di - do))

    # Table
    print(f"{'delta_bicycle':>14} {'inner':>8} {'outer':>8} {'diff':>7} {'R (m)':>8}")
    for dd in (5, 10, 15, 20, 25, 30, 35):
        di, do = ackermann_angles(math.radians(dd))
        R = L / math.tan(math.radians(dd))
        print(f"{dd:>13}° {math.degrees(di):>7.2f}° {math.degrees(do):>7.2f}° "
              f"{math.degrees(di - do):>6.2f}° {R:>8.2f}")

    v = 5.0
    vi, vo = rear_wheel_speeds(v, math.radians(20))
    print(f"\nAt v={v} m/s, delta=20°: v_inner={vi:.3f}, v_outer={vo:.3f} m/s")

    # Plot
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(deltas_deg, d_in, label='inner wheel')
    ax[0].plot(deltas_deg, d_out, label='outer wheel')
    ax[0].plot(deltas_deg, deltas_deg, '--', label='bicycle model (delta)')
    ax[0].set_xlabel('bicycle steering angle (deg)')
    ax[0].set_ylabel('wheel angle (deg)')
    ax[0].set_title('Ackermann wheel angles')
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(deltas_deg, err)
    ax[1].set_xlabel('bicycle steering angle (deg)')
    ax[1].set_ylabel('inner - outer (deg)')
    ax[1].set_title('Ackermann steering difference')
    ax[1].grid(True)

    plt.tight_layout()
    plt.savefig('ackermann_comparison.png', dpi=150)
    print('Saved ackermann_comparison.png')


if __name__ == '__main__':
    main()