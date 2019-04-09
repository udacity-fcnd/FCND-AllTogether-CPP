#include "modules/common/Common.h"
#include "modules/estimation/BaseQuadEstimator.h"

BaseQuadEstimator::BaseQuadEstimator(string config)
{
  _config = config;
  Init();
}

BaseQuadEstimator::~BaseQuadEstimator()
{

}
