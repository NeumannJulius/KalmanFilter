from math import sin, cos
import MatrixHelpers as mh
import numpy as np


class KalmanFilter:
    def __init__(self, dt, u_x, u_y, std_Q, x_std_R, y_std_R):
        """

        Args:
            dt (_type_): sampling time (dt)
            u_x (_type_): acceleration in x-direction
            u_y (_type_): acceleration in y-direction
            std_Q (_type_): process noise
            x_std_R (_type_): standard deviation of the measurement in x-direction
            y_std_R (_type_): standard deviation of the measurement in y-direction
        """

        # Define Parameters

        # Sampling Time
        self.dt = dt

        # Define acceleration inputs
        self.u = [[u_x], [u_y]]

        # set intial state [[x],[y],[vx],[vy]]
        self.x = [[0], [0], [0], [0]]

        # Define the State Transition Matrix A
        self.A = [[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]]

        # Define the Control Input Matrix B
        self.B = [
            [(self.dt**2) / 2, 0],
            [0, (self.dt**2) / 2],
            [self.dt, 0],
            [0, self.dt],
        ]

        # Define Pos Measurement Mapping Matrix
        self.HPos = [[1, 0, 0, 0], [0, 1, 0, 0]]

        # Define Vel Measurement Mapping Matrix
        self.HVel = [[0, 0, 1, 0], [0, 0, 0, 1]]

        self.H = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

        # Initial Process Noise Covariance Matrix Q
        self.Q = mh.matrixMultiplicationVariable(
            [
                [(self.dt**4) / 4, 0, (self.dt**3) / 2, 0],
                [0, (self.dt**4) / 4, 0, (self.dt**3) / 2],
                [(self.dt**3) / 2, 0, self.dt**2, 0],
                [0, (self.dt**3) / 2, 0, self.dt**2],
            ],
            std_Q**2,
        )

        # Initial Measurement Noise Covariance Matrix R
        self.R = [
            [x_std_R**2, 0],
            [0, y_std_R**2],
        ]

        self.R2 = [
            [x_std_R**2, 0, 0, 0],
            [0, y_std_R**2, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]

        # Initial Covariance Matrix P
        self.P = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

    def predict(self):
        # x_k =Ax_(k-1) + Bu_(k-1)
        self.x = mh.matrixAddition(
            mh.matrixMultiplication(self.A, self.x),
            mh.matrixMultiplication(self.B, self.u),
        )

        # Calculate error covariance
        # P= A*P*AT + Q
        self.P = mh.matrixAddition(
            mh.matrixMultiplication(
                mh.matrixMultiplication(self.A, self.P), mh.matrixTranspose(self.A)
            ),
            self.Q,
        )
        return self.x

    def update(self, z, H):
        # S = H*P*H'+R
        S = mh.matrixAddition(
            mh.matrixMultiplication(
                mh.matrixMultiplication(H, self.P), mh.matrixTranspose(H)
            ),
            self.R,
        )

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = mh.matrixMultiplication(
            mh.matrixMultiplication(self.P, mh.matrixTranspose(H)),
            mh.matrixInversion2x2(S),
        )

        self.x = mh.matrixAddition(
            self.x,
            mh.matrixMultiplication(
                K, mh.matrixSubtraction(z, mh.matrixMultiplication(H, self.x))
            ),
        )

        I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

        self.P = mh.matrixMultiplication(
            mh.matrixSubtraction(I, mh.matrixMultiplication(K, H)), self.P
        )
        return self.x

    def update2(self, z):
        S = mh.matrixAddition(
            mh.matrixMultiplication(
                mh.matrixMultiplication(self.H, self.P), mh.matrixTranspose(self.H)
            ),
            self.R2,
        )

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = mh.matrixMultiplication(
            mh.matrixMultiplication(self.P, mh.matrixTranspose(self.H)),
            mh.matrixinversion4x4(S),
        )

        self.x = mh.matrixAddition(
            self.x,
            mh.matrixMultiplication(
                K, mh.matrixSubtraction(z, mh.matrixMultiplication(self.H, self.x))
            ),
        )

        I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]

        self.P = mh.matrixMultiplication(
            mh.matrixSubtraction(I, mh.matrixMultiplication(K, self.H)), self.P
        )
        return self.x


if __name__ == "__main__":
    kf = KalmanFilter(0.1, 1, 1, 0.24, 0.84, 0.84)
    predictions = []
    updates = []
    measurements = []
    for i in range(150):

        prediction = kf.predict()
        # print(prediction)

        predictions.append([prediction[0], prediction[1]])
        z = [
            [i + np.random.normal(0, 10)],
            [i + np.random.normal(0, 10)],
            [1 + np.random.normal(0, 0.1)],
            [1 + np.random.normal(0, 0.1)],
        ]

        measurements.append([z[0], z[1]])
        # update = kf.update(z, kf.HPos)
        update = kf.update2(z)
        # z = [[1], [1]]
        # update = kf.update(z, kf.HVel)
        updates.append([update[0], update[1]])

    import matplotlib.pyplot as plt

    ##plt.plot([i[0] for i in predictions],label="Predictions")
    plt.plot([i[0] for i in predictions], label="Updates", color="blue")
    plt.plot([i[0] for i in measurements], label="Measurements", color="red")
    plt.plot([i[1] for i in predictions], label="Updates_Y", color="green")
    plt.plot([i[1] for i in measurements], label="Measurements_Y", color="orange")
    plt.show()
