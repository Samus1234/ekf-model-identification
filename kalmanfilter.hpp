#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <functional>

class KF
{
    public:

    KF(Eigen::VectorXf x, Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R) : x(x), A(A), B(B), C(C), P(P), Q(Q), R(R)
    {
        n = Q.cols();
        d = R.cols();

        K = Eigen::MatrixXf::Zero(n, d);

        I = Eigen::MatrixXf::Identity(d, d);
    }

    Eigen::VectorXf update(Eigen::VectorXf y, Eigen::VectorXf u)
    {
        x_ = A * x + B * u;
        P_ = A * P * A.transpose() + Q;

        S = C * P_ * C.transpose() + R;
        K = P_ * C.transpose() * (S.llt().solve(I));
        x = x_ + K * (y - C * x_);
        P = P_ - K * C * P_;

        return x;
    }

    private:

    size_t n, d;

    Eigen::MatrixXf A, B, C, P, P_, Q, R, S, K, I;

    Eigen::VectorXf x, x_;

};

class EKF
{
    public:

    EKF(Eigen::VectorXf x, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R, std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys, std::function<Eigen::VectorXf(Eigen::VectorXf)> h)
     : x(x), P(P), Q(Q), R(R), f_sys(f_sys), h(h)
    {
        n = Q.cols();
        d = R.cols();

        K = Eigen::MatrixXf::Zero(n, d);

        I = Eigen::MatrixXf::Identity(d, d);

        F = Eigen::MatrixXf::Zero(n, n);

        H = Eigen::MatrixXf::Zero(d, n);

        epsilon = 1e-4;
    }

    void computeF(Eigen::VectorXf x)
    {
        Eigen::MatrixXf eye = Eigen::MatrixXf::Identity(n, n) * epsilon;

        for (unsigned i = 0; i < n; i++)
        {
            F.col(i) = ( f(x + eye.col(i)) - f(x - eye.col(i)) ) / 2 / epsilon;
        }
    }

    void computeH(Eigen::VectorXf x)
    {
        Eigen::MatrixXf eye = Eigen::MatrixXf::Identity(n, n) * epsilon;

        for (unsigned i = 0; i < n; i++)
        {
            H.col(i) = ( h(x + eye.col(i)) - h(x - eye.col(i)) ) / 2 / epsilon;
        }
    }

    Eigen::VectorXf update(Eigen::VectorXf y, Eigen::VectorXf u)
    {
        f = [&](Eigen::VectorXf x_in) { return f_sys(x_in, u); };

        computeF(x);

        computeH(x);

        x_ = f_sys(x, u);
        P_ = F * P * F.transpose() + Q;

        S = H * P_ * H.transpose() + R;
        K = P_ * H.transpose() * (S.llt().solve(I));
        x = x_ + K * (y - H * x_);
        P = P_ - K * H * P_;

        return x;
    }

    private:

    size_t n, d;

    float epsilon;

    Eigen::MatrixXf F, H, P, P_, Q, R, S, K, I;

    Eigen::VectorXf x, x_;

    std::function<Eigen::VectorXf(Eigen::VectorXf, Eigen::VectorXf)> f_sys;

    std::function<Eigen::VectorXf(Eigen::VectorXf)> f;

    std::function<Eigen::VectorXf(Eigen::VectorXf)> h;
};