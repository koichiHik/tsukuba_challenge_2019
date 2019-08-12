

#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

namespace koichi_robotics_lib
{
class OdomParams
{
public:
  OdomParams()
    : useRightPls(true)
    , useLeftPls(true)
    , distBufferSize(10)
    , imuYawBufferSize(1)
    , whlYawBufferSize(1)
    , rightDistPerRad(1.0)
    , leftDistPerRad(1.0)
    , gyro1BitInRad(1.0)
    , standstillThresh(0.0)
    , cycleTime(0.01)
  {
  }

public:
  bool useRightPls, useLeftPls;
  int distBufferSize, imuYawBufferSize, whlYawBufferSize;
  double rightDistPerRad, leftDistPerRad, gyro1BitInRad;
  double wheelBase, standstillThresh;
  double cycleTime;
};

class OdomPos
{
public:
  OdomPos() : x(0.0), y(0.0), z(0.0), angx(0.0), angy(0.0), angz(0.0), totDist(0.0)
  {
  }

  ~OdomPos()
  {
  }

  void reset()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    angx = 0.0;
    angy = 0.0;
    angz = 0.0;
    totDist = 0.0;
  }

  void copy(OdomPos &pos)
  {
    this->x = pos.x;
    this->y = pos.y;
    this->z = pos.z;
    this->angx = pos.angx;
    this->angy = pos.angy;
    this->angz = pos.angz;
    this->totDist = pos.totDist;
  }

  double x, y, z;
  double angx, angy, angz;
  double totDist;
};

class OdomVel
{
public:
  OdomVel() : vx(0.0), vy(0.0), vz(0.0), wx(0.0), wy(0.0), wz(0.0)
  {
  }

  ~OdomVel()
  {
  }

  void reset()
  {
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    wx = 0.0;
    wy = 0.0;
    wz = 0.0;
  }

  void copy(OdomVel &vel)
  {
    this->vx = vel.vx;
    this->vy = vel.vy;
    this->vz = vel.vz;
    this->wx = vel.wx;
    this->wy = vel.wy;
    this->wz = vel.wz;
  }

  double vx, vy, vz;
  double wx, wy, wz;
};

class Odometry
{
public:
  Odometry();
  ~Odometry();

  bool Initialize(const OdomParams &params);
  bool SetCurrentGyroZ(int gyroZ);
  bool SetCurrentPosition(double rRad, double lRad, double rAngVel, double lAngVel);
  bool CalculateOdometry(OdomPos &odomWhl, OdomPos &odomImu, OdomVel &velWhl, OdomVel &velImu, double dT);
  bool GetCurrentOdometry(OdomPos &odomWhl, OdomPos &odomImu, OdomVel &velWhl, OdomVel &velImu);
  bool Finalize();

private:
  double getImuYawRate();
  double getWhlYawRate(double dT);
  void getIncrDist(double &distIncrAvg, double &distIncrLeft, double &distIncrRight);
  void getWhlBasedVelocity(double &vx, double &vy);
  void getDistBasedVelocity(double &vx, double &vy, double dT);
  void calculatePosition(OdomPos &odom, double distIncr, double yawRate, double dT);
  void calculateWhlVelocity(OdomVel &vel);
  void calculateImuVelocity(OdomVel &vel, double dT);
  void updateBuffer(double distIncr, double whlYawRate, double imuYawRate);

private:
  // boost::mutex odomMutex, pulseMutex, gyroMutex;

  OdomParams param;

  OdomPos odomPosWhl, odomPosImu;

  OdomVel odomVelWhl, odomVelImu;

  bool initDone;

  double curRightRad, curLeftRad, rightAngVel, leftAngVel;

  double dT;

  int curGyroZ, prevGyroZ, gyroZFirst100, offsetGyroZ;

  double *distIncrAvgHistory, *yawRateHistoryImu, *yawRateHistoryWhl;
};
}

#endif