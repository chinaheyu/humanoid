import numpy as np


class SingleMINCOTrajectory:
    def __init__(self, X, T) -> None:
        self.T = T
        self.C = self.calculate(len(T), X, T)

    def calculate(self, n, X, T):
        A = np.zeros((6 * n, 6 * n))
        b = np.zeros((6 * n, 1))
        for i in range(n):
            A[i, 6 * i] = 1
            b[i, 0] = X[i]
        for i in range(n):
            for j in range(6):
                A[n + i, 6 * i + j] = np.power(T[i], j)
            b[n + i, 0] = X[i + 1]
        for i in range(0, n - 1):
            for j in range(5):
                A[2 * n + i, 6 * i + j + 1] = np.power(T[i], j) * (j + 1)
            A[2 * n + i, 6 * (i + 1) + 1] = -1
        for i in range(0, n - 1):
            for j in range(4):
                A[3 * n + i - 1, 6 * i + j + 2] = np.power(T[i], j) * (j + 1) * (j + 2)
            A[3 * n + i - 1, 6 * (i + 1) + 2] = -2
        for i in range(0, n - 1):
            for j in range(3):
                A[4 * n + i - 2, 6 * i + j + 3] = np.power(T[i], j) * (j + 1) * (j + 2) * (j + 3)
            A[4 * n + i - 2, 6 * (i + 1) + 3] = -6
        for i in range(0, n - 1):
            for j in range(2):
                A[5 * n + i - 3, 6 * i + j + 4] = np.power(T[i], j) * (j + 1) * (j + 2) * (j + 3) * (j + 4)
            A[5 * n + i - 3, 6 * (i + 1) + 4] = -24
        A[6 * n - 4, 1] = 1
        A[6 * n - 3, 2] = 2
        for j in range(5):
            A[6 * n - 2, 6 * (n - 1) + j + 1] = np.power(T[n - 1], j) * (j + 1)
        for j in range(4):
            A[6 * n - 1, 6 * (n - 1) + j + 2] = np.power(T[n - 1], j) * (j + 1) * (j + 2)

        return np.linalg.pinv(A) @ b

    def plan(self, t):
        i = 0
        while i < len(self.T) - 1:
            if t - self.T[:i].sum() < self.T[i]:
                break
            i += 1
        tp = t - self.T[:i].sum()
        r = np.zeros((3, 1))
        for j in range(3):
            r[j, 0] = np.sum([self.C[6 * i + j + k, 0] * np.power(tp, k) * (1 if j == 0 else np.prod([k + l + 1 for l in range(j)])) for k in range(6 - j)], axis=0)
        return r


class MINCOTrajectory:
    def __init__(self, X, T) -> None:
        """Generate minco trajectories.

        Args:
            X (m, n + 1)
            T (n,)
            where m is the number of joints and n is the number of waypoints.
        """
        self.minco_trajectories = [SingleMINCOTrajectory(X[i, :], T) for i in range(X.shape[0])]
    
    def plan(self, t: float) -> np.ndarray:
        """Plan trajectories.

        Args:
            t (float): Time between 0 and t_final.

        Returns:
            np.ndarray: 3xn matrix, the rows of the matrix are position, velocity and acceleration.
        """
        plans = [self.minco_trajectories[i].plan(t) for i in range(len(self.minco_trajectories))]
        return np.hstack(plans)


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
