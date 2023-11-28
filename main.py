import numpy as np


class KalmanFilter:
    def __init__(self):
        # Initial state (quaternion representation: [q0, q1, q2, q3])
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Initial covariance matrix
        self.P = np.eye(4) * 0.01

        # Process noise covariance
        self.Q = np.eye(4) * 0.01

        # Measurement noise covariance
        self.R = np.eye(4) * 0.01

        # Identity matrix
        self.I = np.eye(4)

        self.F = np.concatenate((0.5*Omega, -0.5*Xi), axis=2)

    def predict(self, omega):
        # Quaternion kinematics (using small angle approximation for simplicity)
        F = np.array([
            [1, -0.5 * omega[0], -0.5 * omega[1], -0.5 * omega[2]],
            [0.5 * omega[0], 1, 0.5 * omega[2], -0.5 * omega[1]],
            [0.5 * omega[1], -0.5 * omega[2], 1, 0.5 * omega[0]],
            [0.5 * omega[2], 0.5 * omega[1], -0.5 * omega[0], 1]
        ])

        # Time update
        self.q = np.dot(F, self.q)
        self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q

    def update(self, z):
        # Kalman gain
        K = np.dot(self.P, np.linalg.inv(self.P + self.R))

        # Measurement update
        self.q = self.q + np.dot(K, (z - self.q))
        self.P = np.dot(self.I - K, self.P)

    def normalize(self):
        self.q = self.q / np.linalg.norm(self.q)


if __name__ == '__main__':
    kf = KalmanFilter()r

    # Simulate a measurement and predict-update cycle
    omega = np.array([0.01, 0.02, 0.03])  # Gyro measurements
    z = np.array([1.0, 0.05, 0.05, 0.05])  # Quaternion measurement (e.g., from a magnetometer)

    kf.predict(omega)
    kf.update(z)
    kf.normalize()
    print(kf.q)
