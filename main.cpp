#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include "kalmanfilter.hpp"
#include "csv.hpp"


#define PI 3.141592653589793238

Eigen::MatrixXf normalRV(size_t d, size_t N)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::default_random_engine generator(seed);

    std::normal_distribution<float> distribution(0.0, 1.0);

    auto normal = [&](float) { return distribution(generator); };

    Eigen::VectorXf n = Eigen::VectorXf::NullaryExpr(N * d, normal);

    return n.matrix().reshaped<Eigen::RowMajor>(d, N);
}


class System
{
    public:

    float b1, b2;

    float Ts;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;

    System()
    {

        Ts = 1e-3;

        b1 = 1.85;

        b2 = 0.75;

        A = Eigen::MatrixXf(4, 4);

        B = Eigen::MatrixXf(4, 2);

        A << 0, 0, 1, 0,
             0, 0, 0, 1,
             0, 0, -b1, 0,
             0, 0, 0, -b2;
        
        B << 0, 0,
             0, 0,
             1, 0,
             0, 1;
    }

    Eigen::VectorXf f_sys(Eigen::VectorXf x, Eigen::VectorXf u)
    {
        return x + Ts*(A * x + B * u);
    }

    Eigen::VectorXf h(Eigen::VectorXf x)
    {
        return x;
    }
};

class EstimatorModel
{
    public:

    float Ts;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;

    EstimatorModel()
    {
        Ts = 1e-3;

        A = Eigen::MatrixXf(6, 6);
        B = Eigen::MatrixXf(6, 2);

        A << 0, 0, 1, 0, 0, 0,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0;
        
        B << 0, 0,
             0, 0,
             1, 0,
             0, 1,
             0, 0,
             0, 0;
    }

    Eigen::VectorXf f_sys(Eigen::VectorXf x, Eigen::VectorXf u)
    {
        A(2, 2) = -x(4);
        A(3, 3) = -x(5);

        return x + Ts*(A * x + B * u);
    }

    Eigen::VectorXf h(Eigen::VectorXf x)
    {
        return x(Eigen::seq(0, 3));
    }
};

class Solver
{
    public:

    Solver(size_t N, Eigen::VectorXf x0, Eigen::VectorXf x0_est, std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys, std::function<Eigen::VectorXf(Eigen::VectorXf)> h, std::shared_ptr<EKF> ekf) : N(N), x0(x0), x0_est(x0_est), f_sys(f_sys), h(h), ekf(ekf)
    {
        n = x0.rows();
        n_est = x0_est.rows();
        noise_pwr = Eigen::MatrixXf(n, n);
        noise_pwr << 0.1, 0, 0, 0,
                     0, 0.1, 0, 0,
                     0, 0, 0.1, 0,
                     0, 0, 0, 0.1;
    }

    void run()
    {
        X = Eigen::MatrixXf::Zero(n, N);

        X_est = Eigen::MatrixXf::Zero(n_est, N);

        X.col(0) = x0;

        X_est.col(0) = x0_est;

        Eigen::VectorXf u = Eigen::VectorXf::Zero(2);

        Eigen::VectorXf y;

        Eigen::MatrixXf noise = noise_pwr * normalRV(n, N);

        for (unsigned i = 0; i < N-1; i++)
        {
            X.col(i + 1) = f_sys(X.col(i), u);
            y = h(X.col(i)) + noise.col(i);

            X_est.col(i + 1) = ekf -> update(y, u);

        }

        CSVData sv1("systemStates.csv", X);
        CSVData sv2("estimatedStates.csv", X_est);
        sv1.writeToCSVfile();
        sv2.writeToCSVfile();
    }

    private:

    size_t n, n_est, N;

    Eigen::VectorXf x0, x0_est;

    Eigen::MatrixXf X, X_est, noise_pwr;

    std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys;

    std::function<Eigen::VectorXf(Eigen::VectorXf)> h;

    std::shared_ptr<EKF> ekf;
};

int main()
{
    System sys;

    EstimatorModel sys_est;

    std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys = [&](Eigen::VectorXf x, Eigen::VectorXf u) { return sys.f_sys(x, u); };

    std::function<Eigen::VectorXf(Eigen::VectorXf)> h = [&](Eigen::VectorXf x) { return sys.h(x); };

    std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys_est = [&](Eigen::VectorXf x, Eigen::VectorXf u) { return sys_est.f_sys(x, u); };

    std::function<Eigen::VectorXf(Eigen::VectorXf)> h_est = [&](Eigen::VectorXf x) { return sys_est.h(x); };

    Eigen::VectorXf x0(4);

    Eigen::VectorXf x0_est(6);

    Eigen::MatrixXf P0 = Eigen::MatrixXf::Identity(6, 6)*1e-2;

    P0(4, 4) = 0.5; P0(5, 5) = 0.5;

    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6)*1e-2;

    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(4, 4)*5e-1;

    x0 << 0, 0, -10, 10;

    x0_est << 0, 0, 0, 0, 2.5, 0.10;

    std::shared_ptr<EKF> ekf = std::make_shared<EKF>(x0_est, P0, Q, R, f_sys_est, h_est);

    Solver solver(2000, x0, x0_est, f_sys, h, ekf);

    solver.run();

    return 0;
}