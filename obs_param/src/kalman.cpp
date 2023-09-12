#include "kalman.h"

void Kalman::Init_Par(Eigen::VectorXd& x, Eigen::MatrixXd& P, Eigen::MatrixXd& R, Eigen::MatrixXd& Q,
                      Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& H, Eigen::VectorXd& u)
{
  m_x = x;
  m_P = P;
  m_R = R;
  m_Q = Q;
  m_A = A;
  m_B = B;
  m_H = H;
  m_u = u;
}

void Kalman::Predict_State()
{
  m_x = m_A * m_x + m_B * m_u;
}

void Kalman::Predict_Cov()
{
  m_P = m_A * m_P * m_A.transpose() + m_Q;
}

Eigen::VectorXd Kalman::Mea_Resd(const Eigen::VectorXd& z)
{
  m_z = z;
  return m_z - m_H * m_x;
}

Eigen::MatrixXd Kalman::Cal_Gain()
{
  return m_P * m_H.transpose() * (m_H * m_P * m_H.transpose() + m_R).inverse();
}

void Kalman::Update_State()
{
  Eigen::MatrixXd kal_gain = Cal_Gain();
  Eigen::VectorXd mea_res = Mea_Resd(m_z);
  m_x += kal_gain * mea_res;
}

void Kalman::Update_Cov()
{
  Eigen::MatrixXd kal_gain = Cal_Gain();
  m_P = m_P - kal_gain * m_H * m_P;
}