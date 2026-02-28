from kiss_slam.sim.simulator import Simulator2D, SimulatorConfig
from kiss_slam.sim.world import World2D


def test_simulator_generates_steps() -> None:
    world = World2D.random(seed=1, n_landmarks=5, xlim=(-5, 5), ylim=(-5, 5))
    sim = Simulator2D(world=world, config=SimulatorConfig(steps=10, dt=0.1), seed=3)
    steps = sim.run()
    assert len(steps) == 10
    assert all(step.control is not None for step in steps)
