import numpy as np
import matplotlib.pyplot as plt
import math

class ExtendedKalmanFilter:
    def __init__(self):
        # State vector: [px, py, vx, vy]
        self.x = np.zeros(4)
        # Covariance matrix
        self.P = np.eye(4) * 0.1
        # Process noise covariance
        self.Q = np.eye(4) * 0.5
        # Lidar measurement noise covariance
        self.R_L = np.array([[0.5, 0], [0, 0.5]])
        # Radar measurement noise covariance
        self.R_R = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])

    def predict(self, dt):
        # State transition matrix (for constant velocity model)
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        
        # Predict state
        self.x = F @ self.x
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_lidar(self, z):
        # Measurement matrix for Lidar
        H_L = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]])
        
        # Predict Lidar measurement
        z_pred = H_L @ self.x
        
        # Calculate innovation
        y = z - z_pred
        
        # Calculate Kalman gain
        S = H_L @ self.P @ H_L.T + self.R_L
        K = self.P @ H_L.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x += K @ y
        self.P = (np.eye(len(self.x)) - K @ H_L) @ self.P

    def update_radar(self, z):
        # Non-linear measurement function for Radar
        px, py, vx, vy = self.x
        rho = np.sqrt(px**2 + py**2)
        
        # Small epsilon to avoid division by zero
        epsilon = 1e-6
        rho = max(rho, epsilon)  # Ensure rho is never zero

        rho_dot = (px * vx + py * vy) / rho

        z_pred = np.array([rho, np.arctan2(py, px), rho_dot])

        # Calculate innovation
        y = z - z_pred
        y[1] = self.normalize_angle(y[1])  # Normalize angle

        # Calculate Jacobian matrix
        H_j = np.array([[px / rho, py / rho, 0, 0],
                        [-py / (rho**2), px / (rho**2), 0, 0],
                        [py * vy - px * vx / (rho**2), px * vy - py * vx / (rho**2), px / rho, py / rho]])

        # Calculate Kalman gain
        S = H_j @ self.P @ H_j.T + self.R_R
        K = self.P @ H_j.T @ np.linalg.inv(S)

        # Update state and covariance
        self.x += K @ y
        self.P = (np.eye(len(self.x)) - K @ H_j) @ self.P

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


def process_measurements(input_file, output_file):
    ekf = ExtendedKalmanFilter()
    results = []
    ground_truth = []
    measurements = []
    estimated_states = []

    last_lidar_time = 0
    last_radar_time = 0

    with open(input_file, 'r') as file:
        for line in file:
            data = line.split()
            if data[0] == 'L':
                meas_px = float(data[1])
                meas_py = float(data[2])
                timestamp = float(data[3])
                gt_px = float(data[4])
                gt_py = float(data[5])
                gt_vx = float(data[6])
                gt_vy = float(data[7])

                # Calculate time interval for Lidar measurement
                dt = timestamp - last_lidar_time
                last_lidar_time = timestamp

                # Predict state
                ekf.predict(dt)
                # Update with Lidar measurement
                ekf.update_lidar(np.array([meas_px, meas_py]))

                # Store data
                ground_truth.append((gt_px, gt_py))
                measurements.append((meas_px, meas_py))
                estimated_states.append((ekf.x[0], ekf.x[1]))

            elif data[0] == 'R':
                meas_rho = float(data[1])
                meas_phi = float(data[2])
                meas_rho_dot = float(data[3])
                timestamp = float(data[4])
                gt_px = float(data[5])
                gt_py = float(data[6])
                gt_vx = float(data[7])
                gt_vy = float(data[8])

                # Calculate time interval for Radar measurement
                dt = timestamp - last_radar_time
                last_radar_time = timestamp

                # Predict state
                ekf.predict(dt)
                # Update with Radar measurement
                ekf.update_radar(np.array([meas_rho, meas_phi, meas_rho_dot]))

                # Store data
                ground_truth.append((gt_px, gt_py))
                measurements.append((meas_rho*math.cos(meas_phi), meas_rho*math.sin(meas_phi)))
                estimated_states.append((ekf.x[0], ekf.x[1]))

            # Store the output: estimated state, measurements, and ground truth
            results.append(f"{ekf.x[0]:.5f} {ekf.x[1]:.5f} {ekf.x[2]:.5f} {ekf.x[3]:.5f} "
                           f"{meas_px if data[0] == 'L' else meas_rho*math.cos(meas_phi):.5f} "
                           f"{meas_py if data[0] == 'L' else meas_rho*math.sin(meas_phi):.5f} "
                           f"{gt_px:.5f} {gt_py:.5f} {gt_vx:.5f} {gt_vy:.5f}")

    # Write results to the output file
    with open(output_file, 'w') as file:
        file.write("\n".join(results))

    return ground_truth, measurements, estimated_states


def plot_results(ground_truth, measurements, estimated_states):
    gt_x, gt_y = zip(*ground_truth)
    meas_x, meas_y = zip(*measurements)
    est_x, est_y = zip(*estimated_states)

    plt.figure(figsize=(10, 6))
    plt.plot(gt_x, gt_y, label='Ground Truth', color='g', linestyle='-', marker='o')
    plt.scatter(meas_x, meas_y, label='Measurements', color='r', marker='o')
    plt.plot(est_x, est_y, label='Estimated States', color='b', linestyle='-', marker='o')

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Extended Kalman Filter Tracking')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    input_file = "C:\\Users\\Dell\\Downloads\\Input.txt"  # Specify the input file name
    output_file = "output.txt"  # Specify the output file name
    ground_truth, measurements, estimated_states = process_measurements(input_file, output_file)
    plot_results(ground_truth, measurements, estimated_states)
