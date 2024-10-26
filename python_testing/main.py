import numpy as np
import matplotlib.pyplot as plt
import random
import math

"""
Simple KF for 2D robot localization

Notes:
    x, y, yaw are relative to the parent frame.
    dx, dy, ddx, ddy, steering_angle are relative to the robot frame, where x is forward
"""

cycle_dt = .01  # Constant time step
real_t = 20  # Time to run the simulation for

WB = .3

# Current state
x = np.array([
        0.,  # x
        0.,  # y
        .2,  # x'
        0.,  # y'
        0.,  # yaw
        .79,  # steering angle
])

# State covariance
P = np.eye(len(x)) * .1


# Simple bicycle model update step
def f(x, dt):
    x_ = x[0]
    y_ = x[1]
    d_x = x[2]
    d_y = x[3]
    yaw = x[4]
    tau = x[5]

    d_yaw = d_x * np.sin(tau) / WB * dt
    yaw_new = yaw + d_yaw

    x_new = x_ + d_x * np.cos(yaw + tau) * dt
    y_new = y_ + d_x * np.sin(yaw + tau) * dt

    return np.array([x_new, y_new, d_x, d_y, yaw_new, tau])


def F(x, dt):
    # Generates a Jacobian matrix representing an Simple bicycle model update step

    d_x = x[2]
    yaw = x[4]
    tau = x[5]

    p_yaw_dx = dt * np.sin(tau) / WB
    p_yaw_tau = dt * d_x * np.cos(tau) / WB

    p_x_dx = dt * np.cos(yaw + tau)
    p_x_yaw = -dt * d_x * np.sin(yaw + tau)
    p_x_tau = -dt * d_x * np.sin(yaw + tau)

    p_y_dx = dt * np.sin(yaw + tau)
    p_y_yaw = dt * d_x * np.cos(yaw + tau)
    p_y_tau = dt * d_x * np.cos(yaw + tau)

    # Jacobian matrix representing an Ackermann update step
    return np.array([
        # x      y       dx          dy      yaw         tau            # OUTPUT
        [1.,     0.,     p_x_dx,     0.,     p_x_yaw,    p_x_tau],      # x
        [0.,     1.,     p_y_dx,     0.,     p_y_yaw,    p_y_tau],      # y
        [0.,     0.,     1.,         0.,     0.,         0.],           # dx
        [0.,     0.,     0.,         1.,     0.,         0.],           # dy
        [0.,     0.,     p_yaw_dx,   0.,     1.,         p_yaw_tau],    # yaw
        [0.,     0.,     0.,         0.,     0.,         1.],           # tau
    ])


def H(x, dt, tr):
    return (tr @ F(x, dt).T).T


# Process noise covariance (update step). Low Q value means high confidence in model
def Q(dt):
    return dt * np.eye(len(x)) * 0.1


def predict(x, P, dt):
    F_k = F(x, dt)
    Q_k = Q(dt)

    # Generate new state prediction with prediction matrix
    state = f(x, dt)

    # Generate new covariance prediction, with additional covariance to account for process noise
    cov = F_k @ P @ F_k.T + Q_k

    return state, cov


def state_update(x, P, dt):
    x_k, P_k = predict(x, P, dt)

    return x_k, P_k


def sensor_update(
        state_,
        variance,
        sensor_state,
        sensor_mul_matrix,
        sensor_jacobian_pre,
        sensor_variance,
        dt
):
    H_k = H(state_, dt, sensor_mul_matrix)
    R_k = sensor_variance
    h = sensor_mul_matrix

    predicted_state, predicted_variance = predict(state_, variance, dt)

    predicted_imu = h @ predicted_state
    real_imu = sensor_state

    print("REAL: ", real_imu)
    print("PRED: ", predicted_imu)

    y_k = real_imu - predicted_imu

    S_k = H_k @ predicted_variance @ H_k.T + R_k
    K_k = predicted_variance @ H_k.T @ np.linalg.pinv(S_k)

    state_ = state_ + K_k @ y_k
    predicted_variance = (np.eye(len(x)) - K_k @ H_k) @ predicted_variance

    return state_, predicted_variance


def fake_imu(state):
    return np.array([
        0.,
        0.,
        state[2],
        state[3],
        state[4],
        0.
    ])

    # return np.array([
    #     0.,
    #     0.,
    #     state[2] + random.gauss(0, 0),
    #     state[3] + random.gauss(0, 0),
    #     state[4] + random.gauss(0, 0),
    #     0.
    # ])


def fake_enc(state):
    return np.array([
        0.,
        0.,
        state[2],
        state[3],
        0.,
        state[5]
    ])
    # return np.array([
    #     0.,
    #     0.,
    #     state[2] + random.gauss(0, 0),
    #     state[3] + random.gauss(0, 0),
    #     0.,
    #     state[5]
    # ])


def main_loop():
    state = x
    variance = P

    sensor_jacobian = {
        "imu": np.array([0., 0., 1., 1., 1., 0.]),
        "enc": np.array([0., 0., 1., 1., 0., 1.]),
    }

    sensor_mul_matrix = {
        "imu": np.array([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0]
        ]),
        "enc": np.array([
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]
        ]),
    }

    sensor_state = {
        "imu": fake_imu,
        "enc": fake_enc,
    }

    sensor_variances = {
        "imu": np.eye(len(x)) * 1.,
        "enc": np.eye(len(x)) * 1.,
    }

    next_state = state
    next_var = variance
    # for i in range(int(real_t / cycle_dt)):
    #     next_state[5] = math.sin(i * cycle_dt / 2) * .79
    #
    #     next_state, next_var = predict(next_state, next_var, cycle_dt)
    #
    #     plt.plot(next_state[0], next_state[1], 'bo')

    # Simulate the state change with F and a dt of .1 100 times using matplotlib
    for i in range(int(real_t / cycle_dt)):
        seed = random.random()

        # Adjust steering angle
        state[5] = math.sin(i * cycle_dt / 2) * .79
        next_state[5] = math.sin(i * cycle_dt / 2) * .79

        next_state, next_var = predict(next_state, next_var, cycle_dt)
        plt.plot(next_state[0], next_state[1], 'bo')

        if seed <= .8:
            state, variance = predict(state, variance, cycle_dt)
        elif seed <= 1.0:
            print("REAA: ", next_state)

            state, variance = sensor_update(
                state,
                variance,
                sensor_state["imu"](next_state),
                sensor_mul_matrix["imu"],
                sensor_jacobian["imu"],
                sensor_variances["imu"],
                cycle_dt
            )
        elif seed < 1.0:
            state, variance = sensor_update(
                state,
                variance,
                sensor_state["enc"](next_state),
                sensor_mul_matrix["enc"],
                sensor_jacobian["enc"],
                sensor_variances["enc"],
                cycle_dt
            )

        plt.plot(state[0], state[1], 'ro')


if __name__ == "__main__":
    main_loop()
    plt.show()
