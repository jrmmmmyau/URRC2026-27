#include <cmath>
#include <Eigen/Dense>

// General EKF: the predict and update steps of Table 7.2,
// for any number of states.
class Ekf
{
public:
  using Vec = Eigen::VectorXd;   // a list of numbers, any length
  using Mat = Eigen::MatrixXd;   // a matrix, any size

  // n           : number of states (2 for the speed EKF, 3 for the integrator)
  // angle_index : which state is an angle, so it gets wrapped; -1 if none
  Ekf(int n, int angle_index = -1)
  : mu(Vec::Zero(n)), Sigma(Mat::Identity(n, n)), angle_index_(angle_index)
  {
  }

  Vec mu;      // best guess
  Mat Sigma;   // uncertainty

  // PREDICT: Table 7.2 lines 6 and 7
  //   motion : how much the state changes (line 6)
  //   G      : derivative of the motion w.r.t. the state (line 3)
  //   R      : new noise from imprecise motion (V M V^T, line 7)
  void predict(const Vec & motion, const Mat & G, const Mat & R)
  {
    mu += motion;                                   // line 6
    wrap_state();
    Sigma = G * Sigma * G.transpose() + R;          // line 7
  }

  // UPDATE: Table 7.2 lines 14 to 17
  //   z         : the measurement
  //   z_hat     : what we expected to measure (line 12)
  //   H         : derivative of the measurement w.r.t. the state (line 13)
  //   Q         : measurement noise (line 8)
  //   angle_row : which row of z is an angle, or -1
  void update(const Vec & z, const Vec & z_hat, const Mat & H, const Mat & Q,
              int angle_row = -1)
  {
    Vec diff = z - z_hat;                           // measured minus expected
    if (angle_row >= 0) {
      diff(angle_row) = wrap(diff(angle_row));
    }

    const Mat S = H * Sigma * H.transpose() + Q;                    // line 14
    const Mat K = Sigma * H.transpose() * S.inverse();              // line 15
    mu += K * diff;                                                 // line 16
    wrap_state();
    const int n = static_cast<int>(mu.size());
    Sigma = (Mat::Identity(n, n) - K * H) * Sigma;                  // line 17
  }

  // Keep an angle between -pi and +pi
  static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

private:
  // Wrap the angle state (if there is one) after every change
  void wrap_state()
  {
    if (angle_index_ >= 0) {
      mu(angle_index_) = wrap(mu(angle_index_));
    }
  }

  int angle_index_;
};