if I > I_max:
    # Current limiting
    V = I_max * R + omega / Kv
else:
    # Normal control here
    V = ...
