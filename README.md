# ekf-model-identification

**A C++ implementation of a general Extended Kalman Filter for use in online or offline model identification needed for model-based controllers (MPC)**

## Example setup and toy model

### Process Model:

$$
\dot{x} = 
\begin{bmatrix}
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
0 & 0 & -b_1 & 0\\
0 & 0 & 0 & -b_2
\end{bmatrix} x +
\begin{bmatrix}
0 & 0\\
0 & 0\\
1 & 0\\
0 & 1
\end{bmatrix}
u
$$

Here, we wish to learn $$ b_1 \hbox{ and } b_2 $$

## External Dependecies
* Linux
* [eigen3][https://eigen.tuxfamily.org/dox/GettingStarted.html]

## Structure
* [kalmanfilter.hpp](https://github.com/sidd-1234/ekf-model-identification/blob/main/kalmanfilter.hpp)
    * Contains an Eigen-based implementation of both linear and extended Kalman filters.
    * Uses lambda expressions and the functional library to pass process and sensor models with numerical Jacobian computation
* [main.cpp](https://github.com/sidd-1234/cpp-torch-from-scratch/blob/main/main.cpp)
    * Contains a toy process model and estimator model for EKF along with a solver.
    * Generates CSV files with the actual system states and estimated states.
* [plot.py](https://github.com/sidd-1234/cpp-torch-from-scratch/blob/main/plot.py)
    * Reads CSV files and generates plots.

## How to run
> . run.sh
