# Normal control here
V = ...

# For predictive
omega = angular velocity measurement
I = V / R - omega / (Kv * R)

# For reactive
I = current measurement
omega = Kv * V - I * R * Kv  # or angular velocity measurement

if I > I_max:
    # Current limiting
    V = I_max * R + omega / Kv
