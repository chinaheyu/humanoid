import numpy as np


class FifthOrderTrajectory:
    def __init__(self, st: np.ndarray, ed: np.ndarray, dt: float) -> None:
        """Generate n fifth-order trajectories.

        Args:
            st (np.ndarray): 3xn matrix, the rows of the matrix are position, velocity and acceleration.
            ed (np.ndarray): 3xn matrix same as st.
            dt (float): duration
        """
        dts = 1.0 / np.power(dt, np.arange(6))
        inv_a = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0.5, 0, 0, 0],
                [
                    -10 * dts[3],
                    -6 * dts[2],
                    -1.5 * dts[1],
                    10 * dts[3],
                    -4 * dts[2],
                    0.5 * dts[1],
                ],
                [
                    15 * dts[4],
                    8 * dts[3],
                    1.5 * dts[2],
                    -15 * dts[4],
                    7 * dts[3],
                    -dts[2],
                ],
                [
                    -6 * dts[5],
                    -3 * dts[4],
                    -0.5 * dts[3],
                    6 * dts[5],
                    -3 * dts[4],
                    0.5 * dts[3],
                ]
            ]
        )
        self.coefficients = inv_a @ np.vstack((st, ed))

    def plan(self, t: float) -> np.ndarray:
        """Plan trajectories.

        Args:
            t (float): Time between 0 and dt.

        Returns:
            np.ndarray: 3xn matrix, the rows of the matrix are position, velocity and acceleration.
        """
        ts = np.power(t, np.arange(6))
        t_matrix = np.array([
            ts,
            [0, 1, 2 * ts[1], 3 * ts[2], 4 * ts[3], 5 * ts[4]],
            [0, 0, 2, 6 * ts[1], 12 * ts[2], 20 * ts[3]],
        ])
        return t_matrix @ self.coefficients
