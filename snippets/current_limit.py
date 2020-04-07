# Normal feedback control
V = K @ (r - x)

# Calculations for predictive current limiting
omega = angular_velocity_measurement
I = V / R - omega / (Kv * R)

# Calculations for reactive current limiting
I = current_measurement
omega = Kv * V - I * R * Kv  # or can be angular velocity measurement

# If predicted/actual current above max, limit current by reducing voltage
if I > I_max:
    V = I_max * R + omega / Kv
