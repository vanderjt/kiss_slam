import numpy as np

from kiss_slam.math_utils import numerical_jacobian, wrap_angle


def test_wrap_angle_range() -> None:
    value = wrap_angle(3 * np.pi)
    assert -np.pi <= value < np.pi
    assert np.isclose(value, -np.pi)


def test_numerical_jacobian_quadratic() -> None:
    def func(x: np.ndarray) -> np.ndarray:
        return np.array([x[0] ** 2 + x[1], x[0] - 3.0 * x[1]])

    jac = numerical_jacobian(func, np.array([2.0, 4.0]))
    expected = np.array([[4.0, 1.0], [1.0, -3.0]])
    assert np.allclose(jac, expected, atol=1e-5)
