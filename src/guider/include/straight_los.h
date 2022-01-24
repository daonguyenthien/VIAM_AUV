#ifndef STRAIGHT_LOS_H
#define STRAIGHT_LOS_H

#include <base_los.h>

class StraightLOS : public BaseLOS
{
public:
  std::vector<double> alpha_P;
  std::vector<double> s;
  unsigned pointId;
  unsigned long numPoints;

  void setupLOS();
  void resetLOS();
  bool runLOS(const double& odomX, const double& odomY);
};

#endif // STRAIGHT_LOS_H
