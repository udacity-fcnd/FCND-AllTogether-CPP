#include "modules/common/Common.h"
#include "modules/simulation/BaseDynamics.h"
#include "modules/math/Random.h"
#include "modules/utility/SimpleConfig.h"
#include "modules/utility/StringUtils.h"
#include "modules/planning/Trajectory.h"

#ifdef _MSC_VER //  visual studio
#pragma warning(disable: 4267 4244 4996)
#endif

using namespace SLR;

BaseDynamics::BaseDynamics(string name)
	:_followedPos(MAX_TRAJECTORY_POINTS),
	 _followedAtt(MAX_TRAJECTORY_POINTS)
{
  _name = name;
  Initialize();
}

int BaseDynamics::Initialize()
{
  ParamsHandle config = SimpleConfig::GetInstance();

  // load in BaseDynamics-specific double-valued settings from the config in your inheritor
	vector<float> tmp;
	if (config->GetFloatVector("Sim.xBounds",tmp))
	{
		xMin = tmp[0];
		xMax = tmp[1];
	}
	else
	{
		xMin = -10;
		xMax = 10;
	}

	if (config->GetFloatVector("Sim.yBounds", tmp))
	{
		yMin = tmp[0];
		yMax = tmp[1];
	}
	else
	{
		yMin = -10;
		yMax = 10;
	}

	if (config->GetFloatVector("Sim.zBounds", tmp))
	{
		zMin = tmp[0];
		zMax = tmp[1];
	}
	else
	{
		zMin = -20;
		zMax = 0;
	}

	_followedPos.reset();
	_followedAtt.reset();

  return 1;
}

void BaseDynamics::ResetState(V3F newPos, V3F newVel, Quaternion<float> newAtt, V3F newOmega)
{
  omega = newOmega;
  pos = newPos;
  vel = newVel;
  quat = newAtt;
}

GlobalPose BaseDynamics::GenerateGP(void)
{
  GlobalPose p;
  p.pos = pos;
  p.q = quat;
  return p;
}

bool BaseDynamics::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');
  if (ToUpper(leftPart) == ToUpper(_name))
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    GETTER_HELPER("POS.X", pos.x);
    GETTER_HELPER("POS.Y", pos.y);
    GETTER_HELPER("POS.Z", pos.z);
    GETTER_HELPER("VEL.X", vel.x);
    GETTER_HELPER("VEL.Y", vel.y);
    GETTER_HELPER("VEL.Z", vel.z);
    GETTER_HELPER("YAW", quat.ToEulerYPR()[0]);
    GETTER_HELPER("PITCH", quat.ToEulerYPR()[1]);
    GETTER_HELPER("ROLL", quat.ToEulerYPR()[2]);
    GETTER_HELPER("OMEGA.X", omega.x);
    GETTER_HELPER("OMEGA.Y", omega.y);
    GETTER_HELPER("OMEGA.Z", omega.z);
		GETTER_HELPER("ACC.X", acc.x);
		GETTER_HELPER("ACC.Y", acc.y);
		GETTER_HELPER("ACC.Z", acc.z);
#undef GETTER_HELPER
  }
  return false;
}

vector<string> BaseDynamics::GetFields() const
{
  vector<string> ret;
  ret.push_back(_name + ".Pos.X");
  ret.push_back(_name + ".Pos.Y");
  ret.push_back(_name + ".Pos.Z");
  ret.push_back(_name + ".Vel.X");
  ret.push_back(_name + ".Vel.Y");
  ret.push_back(_name + ".Vel.Z");
  ret.push_back(_name + ".Yaw");
  ret.push_back(_name + ".Pitch");
  ret.push_back(_name + ".Roll");
  ret.push_back(_name + ".Omega.X");
  ret.push_back(_name + ".Omega.Y");
  ret.push_back(_name + ".Omega.Z");
	ret.push_back(_name + ".Acc.X");
	ret.push_back(_name + ".Acc.Y");
	ret.push_back(_name + ".Acc.Z");
  return ret;
}
