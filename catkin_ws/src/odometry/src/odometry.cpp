
// System Header
#include <math.h>
#include <algorithm>
#include <iostream>

// Original
#include "odometry.h"

namespace koichi_robotics_lib
{
Odometry::Odometry()
  : initDone(false)
  , curRightRad(0)
  , curLeftRad(0)
  , curGyroZ(0)
  , prevGyroZ(0)
  , gyroZFirst100(0)
  , offsetGyroZ(0)
  , distIncrAvgHistory(nullptr)
  , yawRateHistoryImu(nullptr)
  , yawRateHistoryWhl(nullptr)
{
}

Odometry::~Odometry()
{
}

bool Odometry::Initialize(const OdomParams &params)
{
  param = params;
  this->distIncrAvgHistory = new double[param.distBufferSize];
  this->yawRateHistoryImu = new double[param.imuYawBufferSize];
  this->yawRateHistoryWhl = new double[param.whlYawBufferSize];

  for (int i = 0; i < param.distBufferSize; i++)
  {
    this->distIncrAvgHistory[i] = 0;
  }

  for (int i = 0; i < param.imuYawBufferSize; i++)
  {
    this->yawRateHistoryImu[i] = 0;
  }

  for (int i = 0; i < param.whlYawBufferSize; i++)
  {
    this->yawRateHistoryWhl[i] = 0;
  }

  return true;
}

bool Odometry::SetCurrentGyroZ(int gyroZ)
{
  static int cnt = 0;

  {
    // boost::mutex::scoped_lock(gyroMutex);

    if (cnt >= 200)
    {
      initDone = true;
      offsetGyroZ = gyroZFirst100 / 100;
      prevGyroZ = curGyroZ;
      curGyroZ = gyroZ;
    }
    else if (100 <= cnt && cnt < 200)
    {
      gyroZFirst100 += gyroZ;
      cnt++;
    }
    else
    {
      cnt++;
    }
  }

  return true;
}

bool Odometry::SetCurrentPosition(double rRad, double lRad, double rAngVel, double lAngVel)
{
  {
    // boost::mutex::scoped_lock(pulseMutex);
    curRightRad = rRad;
    curLeftRad = lRad;
    rightAngVel = rAngVel;
    leftAngVel = lAngVel;
  }

  return true;
}

void Odometry::calculatePosition(OdomPos &odom, double distIncr, double yawRate, double dT)
{
  double prevYaw = odom.angz;
  odom.angz = prevYaw + yawRate * dT;

  double timeAvgYaw = (odom.angz + prevYaw) / 2.0;
  odom.x = odom.x + cos(timeAvgYaw) * distIncr;
  odom.y = odom.y + sin(timeAvgYaw) * distIncr;
  odom.totDist = odom.totDist + distIncr;
}

bool Odometry::CalculateOdometry(OdomPos &odomWhl, OdomPos &odomImu, OdomVel &velWhl, OdomVel &velImu, double dT)
{
  if (!initDone)
  {
    odomWhl.reset();
    odomImu.reset();
    velWhl.reset();
    velImu.reset();
    return false;
  }

  {
    // boost::mutex::scoped_lock(odomMutex);
    odomWhl.copy(this->odomPosWhl);
    odomImu.copy(this->odomPosImu);
    velWhl.copy(this->odomVelWhl);
    velImu.copy(this->odomVelImu);
  }

  double distIncrAvg, distIncrLeft, distIncrRight;
  getIncrDist(distIncrAvg, distIncrLeft, distIncrRight);
  double absdist = std::abs(distIncrLeft) + std::abs(distIncrRight);

  bool standstill = false;
  if (absdist < param.standstillThresh)
  {
    standstill = true;
  }

  // Calculate Position & Velocity Based on Yawrate from IMU.
  double imuYawRate = getImuYawRate();
  if (standstill)
  {
    calculatePosition(odomImu, 0.0, 0.0, dT);
  }
  else
  {
    calculatePosition(odomImu, distIncrAvg, imuYawRate, dT);
  }

  // Calculate Position & Velocity Based on Yawrate from Wheel diff.
  double whlYawRate = getWhlYawRate(dT);
  if (standstill)
  {
    calculatePosition(odomWhl, 0.0, 0.0, dT);
  }
  else
  {
    calculatePosition(odomWhl, distIncrAvg, whlYawRate, dT);
  }

  // Velocity gets caluclated after buffer update.
  updateBuffer(distIncrAvg, whlYawRate, imuYawRate);
  calculateImuVelocity(velImu, dT);
  calculateWhlVelocity(velWhl);

  // Update Position.
  {
    // boost::mutex::scoped_lock(odomMutex);
    this->odomPosWhl.copy(odomWhl);
    this->odomPosImu.copy(odomImu);
    this->odomVelWhl.copy(velWhl);
    this->odomVelImu.copy(velImu);
  }

  return true;
}

void Odometry::getIncrDist(double &distIncrAvg, double &distIncrLeft, double &distIncrRight)
{
  static double prevLeftRad = curLeftRad;
  static double prevRightRad = curRightRad;
  {
    // boost::mutex::scoped_lock(pulseMutex);
    distIncrLeft = (curLeftRad - prevLeftRad) * param.leftDistPerRad;
    prevLeftRad = curLeftRad;
    distIncrRight = (curRightRad - prevRightRad) * param.rightDistPerRad;
    prevRightRad = curRightRad;
  }

  if (param.useRightPls && param.useLeftPls)
  {
    distIncrAvg = (distIncrLeft + distIncrRight) / 2.0;
  }
  else if (param.useRightPls)
  {
    distIncrAvg = distIncrRight;
  }
  else if (param.useLeftPls)
  {
    distIncrAvg = distIncrLeft;
  }
}

double Odometry::getImuYawRate()
{
  double omegaZCorrected = 0.0;
  {
    // boost::mutex::scoped_lock(gyroMutex);
    omegaZCorrected = (curGyroZ - offsetGyroZ) * param.gyro1BitInRad;
  }
  return omegaZCorrected;
}

double Odometry::getWhlYawRate(double dT)
{
  double wz = 0.0, dl = 0.0, dr = 0.0;

  {
    // boost::mutex::scoped_lock(pulseMutex);
    static double pastRightRad = curRightRad;
    static double pastLeftRad = curLeftRad;
    dl = (curLeftRad - pastLeftRad) * param.leftDistPerRad;
    dr = (curRightRad - pastRightRad) * param.rightDistPerRad;
    pastRightRad = curRightRad;
    pastLeftRad = curLeftRad;
  }

  wz = (dr - dl) / (param.wheelBase * dT);
  return wz;
}

void Odometry::getDistBasedVelocity(double &vx, double &vy, double dT)
{
  double sumDist = 0;
  for (int i = 0; i < param.distBufferSize; i++)
  {
    sumDist += this->distIncrAvgHistory[i];
  }

  vx = sumDist / (double)(param.distBufferSize * dT);
  vy = 0;
}

void Odometry::getWhlBasedVelocity(double &vx, double &vy)
{
  double vxl = 0.0, vxr = 0.0;
  {
    // boost::mutex::scoped_lock(pulseMutex);
    double vxl = leftAngVel * param.leftDistPerRad;
    double vxr = rightAngVel * param.rightDistPerRad;
  }
  vx = (vxl + vxr) / 2.0;
  vy = 0;
}

void Odometry::updateBuffer(double distIncr, double whlYawRate, double imuYawRate)
{
  static int distBufIdx = 0, whlYawBuffIdx = 0, imuYawBuffIdx = 0;

  this->distIncrAvgHistory[distBufIdx] = distIncr;
  distBufIdx = (++distBufIdx) % param.distBufferSize;

  this->yawRateHistoryImu[imuYawBuffIdx];
  imuYawBuffIdx = (++imuYawBuffIdx) % param.imuYawBufferSize;

  this->yawRateHistoryWhl[whlYawBuffIdx];
  whlYawBuffIdx = (++whlYawBuffIdx) % param.whlYawBufferSize;
}

void Odometry::calculateWhlVelocity(OdomVel &vel)
{
  double vx, vy;
  getWhlBasedVelocity(vx, vy);
  vel.vx = vx;
  vel.vy = vy;
  vel.vz = 0.0;

  // Yawrate Filtering.
  {
    double sumz = 0;
    for (int i = 0; i < param.whlYawBufferSize; i++)
    {
      sumz += this->yawRateHistoryWhl[i];
    }
    vel.wx = 0.0;
    vel.wy = 0.0;
    vel.wz = sumz / (double)(param.whlYawBufferSize);
  }
}

void Odometry::calculateImuVelocity(OdomVel &vel, double dT)
{
  // Velocity Filtering.
  double vx, vy;
  getDistBasedVelocity(vx, vy, dT);
  vel.vx = vx;
  vel.vy = vy;
  vel.vz = 0.0;

  // Yawrate Filtering.
  {
    double sumz = 0;
    for (int i = 0; i < param.imuYawBufferSize; i++)
    {
      sumz += this->yawRateHistoryImu[i];
    }
    vel.wx = 0.0;
    vel.wy = 0.0;
    vel.wz = sumz / (double)(param.imuYawBufferSize);
  }
}

bool Odometry::GetCurrentOdometry(OdomPos &odomWhl, OdomPos &odomImu, OdomVel &velWhl, OdomVel &velImu)
{
  {
    // boost::mutex::scoped_lock(odomMutex);
    odomWhl.copy(this->odomPosWhl);
    odomImu.copy(this->odomPosImu);
    velWhl.copy(this->odomVelWhl);
    velImu.copy(this->odomVelImu);
  }

  return true;
}

bool Odometry::Finalize()
{
  delete[] this->distIncrAvgHistory;
  delete[] this->yawRateHistoryImu;
  delete[] this->yawRateHistoryWhl;

  return true;
}
}