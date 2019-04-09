#include "modules/common/Common.h"
#include "modules/simulation/Simulator.h"
#include "modules/utility/SimpleConfig.h"
#include "modules/simulation/BaseDynamics.h"
using namespace SLR;

Simulator::Simulator()
{
	Reset();
}

void Simulator::Reset()
{
	ParamsHandle config = SimpleConfig::GetInstance();

	// TODO: parameters

	for (unsigned int i = 0; i < _vehicles.size(); i++)
	{
		_vehicles[i]->Initialize();
	}
}

void Simulator::Run(float dt)
{
	// todo: step in maximally acceptable time steps
}

void Simulator::AddVehicle(shared_ptr<BaseDynamics> vehicle)
{
	_vehicles.push_back(vehicle);
}
