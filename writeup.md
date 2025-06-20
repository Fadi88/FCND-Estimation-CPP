# Estimation Project Write-up

## Introduction

This document details the development and implementation of the estimation portion of a quadrotor controller for the CPP simulator. This project involved building an **Extended Kalman Filter (EKF)** to estimate the quadrotor's state, integrating various sensor readings, and finally, incorporating a custom-tuned controller from a previous project to achieve stable flight based on estimated state information.

The project followed a structured approach, starting with sensor noise characterization and progressively building up the EKF with attitude estimation, prediction steps, and sensor updates (magnetometer and GPS). The final stage involved integrating and re-tuning a previously developed controller to work effectively with the estimated state.

---

## Setup and Project Structure

The project utilized the C++ development environment established in the Controls C++ project. The codebase was cloned from `https://github.com/udacity/FCND-Estimation-CPP.git` and imported into an IDE.

Key files and directories for this project included:

* `QuadEstimatorEKF.cpp`: The core file where the Extended Kalman Filter (EKF) was implemented.
* `QuadEstimatorEKF.txt`: Configuration file for tuning EKF parameters.
* `config/`: This directory contained various simulation configuration files (e.g., `06_NoisySensors.txt`, `07_AttitudeEstimation.txt`, etc.), as well as the controller and estimator parameter files.
* `config/log/Graph1.txt` and `config/log/Graph2.txt`: Log files used for collecting sensor data in Step 1.

The simulator provided useful debug variables for monitoring the estimator's performance:

* `Quad.Est.E.*`: Error in estimated state variables (e.g., `Quad.Est.E.X` for X position error).
* `Quad.Est.S.*`: Estimated standard deviations of state variables from the covariance matrix (e.g., `Quad.Est.S.X` for X position standard deviation).
* `Quad.Est.D.*`: Miscellaneous debug variables.

---

## The Tasks and Implementation Details

### Step 1: Sensor Noise

**Objective:** Collect simulated noisy sensor data and estimate the **standard deviations** of the GPS X signal and IMU Accelerometer X signal.

**Implementation:**
This step involved running `scenario 06_NoisySensors`. The simulator logged GPS X position data to `config/log/Graph1.txt` and Accelerometer X data to `config/log/Graph2.txt`.

* **Data Processing:** The logged CSV files were processed to calculate the standard deviation for both GPS X and Accelerometer X. I loaded the data into a spreadsheet program and calculated the standard deviation using the `STDEV.S` function for each dataset.
* **Parameter Tuning:** The calculated standard deviation values were then plugged into `config/6_Sensornoise.txt` for `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY`.

**Results and Success Criteria:**
Upon re-running `scenario 06_NoisySensors` with the updated parameters, the dashed lines representing the +/- 1 sigma bounds for GPS X and Accelerometer X measurements turned green. This indicated that approximately 68% of the measurement points fell within these bounds, confirming accurate standard deviation estimation.

* `MeasuredStdDev_GPSPosXY`: [Your calculated value here]
* `MeasuredStdDev_AccelXY`: [Your calculated value here]

---

### Step 2: Attitude Estimation

**Objective:** Improve the complementary filter-type attitude filter in `UpdateFromIMU()` in `QuadEstimatorEKF.cpp` to reduce errors in estimated Euler angles.

**Implementation:**
`scenario 07_AttitudeEstimation` was used, which only utilized an ideal IMU. Initially, significant errors were observed in attitude estimation.

* **Non-linear Complementary Filter:** The `UpdateFromIMU()` function in `QuadEstimatorEKF.cpp` was modified to implement a more robust non-linear rate gyro attitude integration scheme. This involved using **quaternions** for attitude representation and integration, as referenced in Section 7.1.2 of the "Estimation for Quadrotors" document. The previous linear scheme was replaced with a quaternion-based approach, where accelerometer readings were used to correct the integrated gyroscope attitude, ensuring a more accurate and stable attitude estimate.

**Results and Success Criteria:**
After implementing the improved scheme, the errors in each of the estimated Euler angles were significantly reduced. The success criteria of maintaining attitude errors within **0.1 rad** for each Euler angle for at least 3 seconds were met. The top graph showing error plots clearly demonstrated a reduction in oscillations and a quick convergence to near-zero error compared to the initial linear scheme.

---

### Step 3: Prediction Step

**Objective:** Implement the state prediction step and predict state covariance forward, tuning process noise parameters.

**Implementation:**

#### State Prediction (`PredictState()`):
`scenario 08_PredictState` was used with a perfect IMU. Initially, the estimated state did not track the true state.

* The `PredictState()` function in `QuadEstimatorEKF.cpp` was implemented to predict the quadrotor's state forward based on the control inputs (accelerations) and the current estimated state. The state vector, including position, velocity, and attitude, was propagated forward using the kinematic equations. The control input, which is the acceleration, was directly integrated into the velocity and position components.

**Results (State Prediction):**
After implementation, the estimator state successfully tracked the actual state with only slow, reasonable drift, as observed in `scenario 08_PredictState`.

#### Covariance Prediction (`Predict()` and `GetRbgPrime()`):
`scenario 09_PredictionCov` introduced a realistic IMU with noise, demonstrating growing errors that were not captured by the initial estimated covariance.

* `GetRbgPrime()`: The partial derivative of the body-to-global rotation matrix was calculated and implemented in the `GetRbgPrime()` function in `QuadEstimatorEKF.cpp`. This matrix is crucial for correctly propagating the covariance. Careful attention was paid to correctly setting all parts of the matrix.
* `Predict()`: The remaining part of the `Predict()` function was implemented to predict the state covariance matrix forward. This involved using the system dynamics Jacobian and process noise covariance.
* **Process Noise Tuning:** The parameters `QPosXYStd` and `QVelXYStd` in `QuadEstimatorEKF.txt` were tuned to accurately capture the magnitude of the growing position and velocity errors observed in `scenario 09_PredictionCov` over a short prediction period (one second). I iteratively adjusted these values, observing how the white bounds (estimated standard deviation) in the plots aligned with the spread of the 10 prediction-only estimates. The goal was to ensure the covariance grew at a similar rate to the actual errors.

**Results (Covariance Prediction):**
The tuned process noise parameters resulted in the estimated covariance (white bounds) closely matching the spread of the position and velocity estimates, particularly in the initial phase of the simulation, indicating a good model of the error growth. The plots in `scenario 09_PredictionCov` showed that the white bounds representing the estimated standard deviation now expanded at a rate consistent with the observed spread of the 10 quadcopter predictions.

* `QPosXYStd`: [Your final tuned value here]
* `QVelXYStd`: [Your final tuned value here]

---

### Step 4: Magnetometer Update

**Objective:** Integrate magnetometer readings to improve the filter's performance in estimating the vehicle's heading (yaw).

**Implementation:**
`scenario 10_MagUpdate` was used. Initially, the estimated yaw drifted significantly, and its standard deviation increased.

* `QYawStd` Tuning (Pre-update): The `QYawStd` parameter in `QuadEstimatorEKF.txt` was initially tuned to capture the magnitude of the yaw drift before the magnetometer update was implemented.
* `UpdateFromMag()` Implementation: The `UpdateFromMag()` function in `QuadEstimatorEKF.cpp` was implemented to incorporate the magnetometer readings into the EKF. This involved comparing the estimated magnetic field vector (derived from the estimated attitude) with the measured magnetic field vector. The Jacobian of this measurement model was computed, and the standard EKF update equations were applied to correct the state and covariance.
* `QYawStd` Tuning (Post-update): After implementing the magnetometer update, `QYawStd` was re-tuned to better balance between long-term drift correction and short-term noise from the magnetometer.

**Results and Success Criteria:**
Following the implementation and tuning, the estimated yaw error (`quad.est.e.yaw`) remained within **0.1 rad** for at least 10 seconds of the simulation in `scenario 10_MagUpdate`. The estimated standard deviation also accurately captured the error, indicating effective yaw estimation. Before the update, the yaw error rapidly diverged; after the update, the error was tightly bound around zero, and the estimated standard deviation effectively contained the small fluctuations.

* `QYawStd` (final value): [Your final tuned value here]

---

### Step 5: Closed Loop + GPS Update

**Objective:** Implement the EKF GPS Update and achieve an estimated position error of less than 1m for the entire simulation cycle.

**Implementation:**
`scenario 11_GPSUpdate` was used. Initially, even with an ideal estimator and IMU, position and velocity errors drifted.

* **Transition to Your Estimator:** `Quad.UseIdealEstimator` in `config/11_GPSUpdate.txt` was set to `0` to use the developed estimator instead of the ideal one.
* **Realistic IMU:** The lines related to ideal IMU settings (`#SimIMU.AccelStd = 0,0,0` and `#SimIMU.GyroStd = 0,0,0`) in `config/11_GPSUpdate.txt` were commented out to introduce realistic IMU noise.
* **Process Noise Tuning:** The process noise model in `QuadEstimatorEKF.txt` was further tuned (especially `QPosXYStd` and `QVelXYStd`) to approximately capture the error observed with the estimated uncertainty (standard deviation) of the filter.
* `UpdateFromGPS()` Implementation: The `UpdateFromGPS()` function in `QuadEstimatorEKF.cpp` was implemented to incorporate GPS measurements. This involved defining the measurement model for GPS (position), computing its Jacobian, and applying the Kalman update equations. The measurement residual was calculated as the difference between the measured GPS position and the estimated position. This was then used in the Kalman update to correct the state and its covariance.

**Results and Success Criteria:**
After implementing and tuning the GPS update, the quadrotor successfully completed the entire simulation cycle with an estimated position error of less than **1m**. This was indicated by the green box over the bottom graph in `scenario 11_GPSUpdate`. Initial GPS updates sometimes caused instability if the measurement noise was set too low or the process noise was not adequately tuned. Iterative adjustments were necessary to find a balance where the GPS corrections effectively stabilized the position estimate without introducing excessive jitter.

---

### Step 6: Adding Your Controller

**Objective:** Integrate the custom controller from the previous project and re-tune it to achieve stable flight with an estimated state, completing the simulation cycle with estimated position error of less than 1m.

**Implementation:**

* **Controller and Parameters Swap:**
    * `QuadController.cpp` was replaced with the custom controller developed in the last project.
    * `config/QuadControlParams.txt` was replaced with the control parameters derived from the last project.
* **Controller De-tuning:** Upon running `scenario 11_GPSUpdate` with the custom controller and estimator, it was observed that the quadrotor might crash due to the differences in flying with an estimated state compared to an ideal pose.
    * The position and velocity gains in `config/QuadControlParams.txt` were de-tuned by approximately **[Your percentage, e.g., 30]%** to stabilize the flight.
    * I initially reverted to ideal sensors (temporarily uncommenting the ideal IMU lines in `11_GPSUpdate.txt`) to isolate the controller tuning from sensor noise. Once stable under ideal sensing, I re-enabled realistic IMU noise and made final adjustments.

**Results and Success Criteria:**
After de-tuning the controller, the quadrotor was able to complete the entire `scenario 11_GPSUpdate` simulation cycle with an estimated position error of less than **1m**, demonstrating successful integration of the custom controller with the developed EKF. The quadrotor maintained a stable flight path throughout the simulation, successfully reaching its waypoints. The estimated position error remained consistently below 1 meter, visible by the green success indicator on the plot.

---

## Tips and Tricks Applied

Throughout the project, several tips and insights were valuable:

* `transposeInPlace()`: The `Matrix.transposeInPlace()` function was consistently used for efficient matrix transposition.
* "Estimation for Quadrotors" Document: This document proved to be an invaluable resource, providing detailed mathematical breakdowns for the core elements of the EKF, including non-linear complementary filters for attitude, transition models, partial derivatives, and measurement updates for magnetometers and GPS. Specifically, sections 7.1.2, 7.2, 7.3.1, and 7.3.2 were frequently referenced.
* **Iterative Tuning:** For parameters like process noise and controller gains, an iterative approach of making small adjustments and observing simulation results was crucial for achieving optimal performance.
* **Analyzing Debug Variables:** Utilizing `Quad.Est.E.*`, `Quad.Est.S.*`, and `Quad.Est.D.*` variables in the plots was essential for diagnosing filter behavior and understanding error propagation.

---

## Additional Contributions

During the development process on macOS, a specific bug was identified and fixed within the project's codebase.
To contribute this fix back to the community and improve the project for other macOS users, a **Merge Request (MR)** was created on the main repository. This ensures that the fix is available to all users and enhances the overall stability and compatibility of the simulation environment on macOS.

---

## Conclusion

This project successfully implemented an Extended Kalman Filter for quadrotor state estimation within the CPP simulator. Each step, from sensor noise characterization to integrating various sensor updates (IMU, Magnetometer, GPS), contributed to building a robust estimator. The final integration and re-tuning of a custom controller demonstrated the ability to achieve stable flight based on estimated state information. The project provided a comprehensive understanding of EKF principles and their practical application in robotics.
