## README

### Portfolio Exam 2: Sensor Fusion – Kalman and Particle Filter

#### Overview
This notebook contains implementations of the Kalman Filter and Particle Filter for simulating and estimating the positions and velocities of balls in motion, based on noisy observations.

### Task P2.1: Kalman Filter

**Description:**
- The Kalman Filter is used to estimate the position and velocity of a single ball.
- The simulation includes generating a true trajectory, adding noise to simulate observations, and applying the Kalman Filter to estimate the true states.
- Results include RMSE calculations for position and velocity, and graphical plots of true paths, noisy observations, and Kalman Filter estimates.

### Task P2.2: Particle Filter

**Description:**
- The Particle Filter is used to estimate the positions and velocities of two balls flying simultaneously.
- The simulation includes generating true trajectories for both balls, adding noise to simulate observations, and applying the Particle Filter to estimate the true states.
- Results include RMSE calculations for positions and velocities, and graphical plots of true paths, noisy observations, and Particle Filter estimates.

**Requirements:**
- Python 3.x
- `numpy` library
- `plotly` library

**Install necessary libraries using:**
- pip install numpy plotly

**Usage:**
1. Adjust the parameters as needed within the code.
2. Run the code cells in order.
3. Review the RMSE results and graphical plots to evaluate the filter performance.
