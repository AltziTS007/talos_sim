#ifndef _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_
#define _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_

#include <laser_odometry_core/laser_odometry_base.h>

namespace rf2o {
class CLaserOdometry2D;
}

namespace laser_odometry
{

class LaserOdometryRf2o : public LaserOdometryBase
{
  using Base = LaserOdometryBase;

public:

  LaserOdometryRf2o()  = default;
  ~LaserOdometryRf2o() = default;

  OdomType odomType() const noexcept override;

protected:

  bool processImpl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                   const Transform& /*prediction*/) override;

protected:

  rf2o::CLaserOdometry2D rf2o_;

  bool configureImpl() override;

  bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_ */
