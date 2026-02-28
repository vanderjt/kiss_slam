from kiss_slam.sim.simulator import SimConfig, Simulator
from kiss_slam.sim.world import World2D


def test_simulator_generates_steps() -> None:
    world = World2D.random(seed=1, n_landmarks=5, xlim=(-5, 5), ylim=(-5, 5))
    sim = Simulator(world=world, config=SimConfig(steps=10, dt=0.1), seed=3)
    steps = sim.run()
    assert len(steps) == 10
    assert all(step_output[0] is not None for step_output in steps)


def test_simulator_reset_reproducible() -> None:
    world = World2D.random(seed=2, n_landmarks=10, xlim=(-8, 8), ylim=(-8, 8))
    sim = Simulator(world=world, config=SimConfig(trajectory_mode="figure_eight"), seed=5)

    rollout_a = sim.run(5)
    sim.reset(seed=5)
    rollout_b = sim.run(5)

    for (ua, _, _, pa), (ub, _, _, pb) in zip(rollout_a, rollout_b):
        assert ua.v == ub.v
        assert ua.w == ub.w
        assert pa.x == pb.x
        assert pa.y == pb.y
        assert pa.yaw == pb.yaw


def test_simulator_dropout() -> None:
    world = World2D.pattern("grid", n_landmarks=50, xlim=(-5, 5), ylim=(-5, 5), seed=0)
    sim = Simulator(
        world=world,
        config=SimConfig(sensor_range=20.0, fov_rad=6.28, measurement_dropout_prob=1.0),
        seed=7,
    )
    _, _, measurements, _ = sim.step()
    assert measurements == []
