# Configuration tuning guide

This guide gives practical starting points for EKF-SLAM noise tuning.

## Q/R quick reference

- `Q`: process (control) covariance on `[v, w]`.
- `R`: measurement covariance on `[range, bearing]`.

```python
import numpy as np

# Example A: indoor differential drive, good wheel odometry + fiducials
Q = np.diag([0.03**2, 0.02**2])
R = np.diag([0.10**2, np.deg2rad(2.0) ** 2])

# Example B: slippery floor / aggressive turns
Q = np.diag([0.10**2, 0.08**2])
R = np.diag([0.20**2, np.deg2rad(4.0) ** 2])

# Example C: long-range noisy detector
Q = np.diag([0.06**2, 0.05**2])
R = np.diag([0.40**2, np.deg2rad(8.0) ** 2])
```

Start larger, then reduce if estimates are too sluggish.

## Effect of odometry biases

Simulator and real robots often have constant bias terms:
- `v` bias causes global position drift.
- `w` bias causes heading drift and curved trajectories.

In `kiss_slam.sim.SimConfig`, these are `control_bias=(bias_v, bias_w)`.
If your real robot has bias and your model does not account for it:
- innovation norms increase,
- landmarks “drag” to compensate,
- NIS grows and consistency degrades.

Mitigations:
- increase `Q` to reflect unmodeled bias,
- calibrate odometry scale and yaw-rate bias offline,
- extend state to include bias terms (advanced).

## Measurement dropout behavior

Dropout means missed observations of otherwise visible landmarks.

In simulator: `measurement_dropout_prob`.

Expected effects:
- short dropouts: mostly harmless, uncertainty grows gradually.
- long dropouts: pose covariance inflates, data association becomes harder.
- high dropout + weak motion excitation: poor observability and drift.

Practical tips:
- avoid overconfident `R` when dropout is significant,
- keep `Q` realistic so uncertainty growth is modeled,
- ensure trajectory has turns (bearing diversity improves corrections),
- use conservative association gates when re-acquiring after dropout.

## Gating (nearest-neighbor)

If using `NearestNeighborAssociator(gate_threshold=...)`:
- lower threshold: safer associations, more new landmarks / misses,
- higher threshold: more matches, more risk of wrong matches.

For 2D measurements, an initial gate around `5.99` (95% chi-square, dof=2) is a reasonable baseline.
