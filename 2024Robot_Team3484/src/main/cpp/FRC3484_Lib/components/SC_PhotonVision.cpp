#include "FRC3484_Lib/components/SC_PhotonVision.h"

#include <cmath>
// #include "wpi/numbers"
#include <numbers>

using namespace SC;
using namespace nt;
using namespace std;

SC_PhotonVision::SC_PhotonVision(double Angle, double LensHeight) {
	angle = Angle;
	lensHeight = LensHeight;
}

SC_PhotonVision::~SC_PhotonVision() {}

bool SC_PhotonVision::HasTarget() {
	return this.camera.HasTargets();
}

double SC_PhotonVision::GetOffsetX() {
	return this->inst->GetNumber("tx", 0.0);
}

// Reports it degress (-29.8, 29.8)
double SC_PhotonVision::GetOffsetY() {
	return this->inst->GetNumber("ty", 0.0);
}

double SC_PhotonVision::GetTargetArea() {
	return this->inst->GetNumber("ta", 0.0);
}

double SC_PhotonVision::GetSkew() {
	return this->inst->GetNumber("ts", 0.0);
}

double SC_PhotonVision::GetPipelineLatency() {
	return this->inst->GetNumber("tl", 0.0);
}

double SC_PhotonVision::GetBBShort() {
	return this->inst->GetNumber("tshort", 0.0);
}

double SC_PhotonVision::GetBBLong() {
	return this->inst->GetNumber("tlong", 0.0);
}

double SC_PhotonVision::GetBBWidth() {
	return this->inst->GetNumber("thor", 0.0);
}

double SC_PhotonVision::GetBBHeight() {
	return this->inst->GetNumber("tvert", 0.0);
}

// double SC_PhotonVision::GetDistanceFromTarget()
// {
// 	return (this->targetHeight - this->lensHeight) / tan((this->angle + this->GetOffsetY()) * (wpi::numbers::pi_v<double> / 180.0));
// }

double SC_PhotonVision::GetDistanceFromTarget() {
	return (this->targetHeight - this->lensHeight) / tan((this->angle + this->GetOffsetY()) * (std::numbers::pi_v<double> / 180.0));
}

int SC_PhotonVision::GetActivePipeline() {
	return ((int)this->inst->GetNumber("getpipe", 0));
}

void SC_PhotonVision::SetLEDMode(SC_LEDMode Mode) {
	this->inst->PutNumber("ledMode", Mode);
}

void SC_PhotonVision::SetDriverCam() {
	this->inst->PutNumber("camMode", 1);
}

void SC_PhotonVision::SetVisionTracking() {
	this->inst->PutNumber("camMode", 0);
}

void SC_PhotonVision::SetPipeline(int Pipeline) {
	this->inst->PutNumber("pipeline", Pipeline);
}

void SC_PhotonVision::SetStreamMode(SC::SC_StreamMode Mode) {
	this->inst->PutNumber("stream", Mode);
}

void SC_PhotonVision::SetCameraAngle(double Angle) {
	this->angle = Angle;
}

void SC_PhotonVision::SetLensHeight(double Height) {
	this->lensHeight = Height;
}

void SC_PhotonVision::SetTargetHeight(double Height) {
	this->targetHeight = Height;
}