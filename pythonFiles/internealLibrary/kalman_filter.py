import numpy as np

std_pos = 91.6#/200
std_acc = 0.22#/200

class KF:
    def __init__(self, initial_x: float,
                 initial_v: float,
                 initial_a: float,
                 accel_variance: float) -> None:
        self._x = np. array([initial_x, initial_v, initial_a])
        self._accel_variance = accel_variance

        self._P = np.eye(3)

    def predict(self, dt: float) -> None:
        # x = A x
        # P = A P At + G Gt a
        A = np.array([[1, dt, dt**2/2],
                      [0, 1, dt],
                      [0, 0, 1]])
        new_x = A.dot(self._x)

        Q = np.array([[std_pos**2, 0, std_pos*std_acc],
                      [         0, 0,               0],
                 [std_acc*std_pos, 0,      std_acc**2]])
        new_P = A.dot(self._P).dot(A.T) + Q# * self._accel_variance

        self._P = new_P
        self._x = new_x

    def update(self, meas_value: float, meas_value2: float, mean_variance: float, mean_variance2: float):
        # y = z - H x
        # S = H P Ht + R
        # K = P Ht S^-1
        # x = x + K y
        # P = (I - K H) * P
        self._x.reshape((3, 1))
        for i in range(2):

            H = np.array([1, 0, 0]).reshape((1, 3))  if i == 0 else np.array([0, 0, 1]).reshape((1, 3))

            z = np.array([meas_value])  if i == 0 else np.array([meas_value2])
            R = np.array([mean_variance]) if i == 0 else np.array([meas_value2])

            y = z - H.dot(self._x)
            S = H.dot(self._P).dot(H.T) + R

            K = self._P.dot(H.T).dot(np.linalg.inv(S))

            new_x = self._x + K.dot(y)
            new_P = (np.eye(3) - K.dot(H)).dot(self._P)

            self._P = new_P
            self._x = new_x

            #print("Runde", i, "H", H, "z", z, "R", R, "y", y, "S", S, "K", K , "new_x", new_x, "new_P", new_P)
        #print("K:", K)
    @property
    def cov(self) -> np.array:
        return self._P

    @property
    def mean(self) -> np.array:
        return self._x

    @property
    def pos(self) -> float:
        return self._x[0]

    @property
    def vel(self) -> float:
        return self._x[1]

    @property
    def acc(self) -> float:
        return self._x[2]
