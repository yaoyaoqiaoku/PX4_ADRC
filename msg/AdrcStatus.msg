uint64 timestamp	# time since system start (microseconds)

# ADRC debug states (per axis: roll, pitch, yaw)
float32[3] z1	# ESO state: angular rate estimate
float32[3] z2	# ESO state: meaning depends on ESO mode
                #  nonlinear ESO (ESO_MODE=0): angular acceleration estimate
                #  linear 2-state ESO (ESO_MODE=1, tau=0): unused (always 0)
                #  augmented 3rd-order ESO (ESO_MODE=1, tau>0): actuator torque state estimate
float32[3] z3	# ESO total disturbance estimate [rad/s^2]
float32[3] v1	# TD reference (rate setpoint tracking)
float32[3] v2	# TD reference derivative (unused in LADRC mode, = 0)
float32[3] u	# control output (after clamp and slew-rate limit)
float32[3] e1	# NLSEF error 1 (setpoint - z1 in LADRC, v1 - z1 in classic)
float32[3] e2	# NLSEF error 2 (always 0 in LADRC mode, v2 - z2 in classic)
float32[3] y	# conditioned feedback (filtered/notched) fed to the ESO
float32[3] ueso	# actuator-shaped control input fed to the ESO
                  #  in augmented ESO mode: mirrors z2 (actuator torque state)
float32[3] integ # LADRC anti-windup integral state
float32[3] z3raw # z3 before saturation (== z3 unless the ESO hit the limit)
float32[3] sp	# smoothed rate setpoint after ADRC_*_SPS (== setpoint if off)
