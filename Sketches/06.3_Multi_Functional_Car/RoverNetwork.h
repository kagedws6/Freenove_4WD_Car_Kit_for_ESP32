#pragma once
void RoverNetworkBegin();
// Called only from the motor-control loop. Returns true while restarting.
bool RoverNetworkService();
