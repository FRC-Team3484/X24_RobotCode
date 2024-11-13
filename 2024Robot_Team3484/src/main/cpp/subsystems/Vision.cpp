#include "subsystems/Vision.h"

using namespace units;

Vision::Vision(double Angle, double LensHeight, double TargetHeight)
:SC_Limelight(Angle, LensHeight) {
    SetCameraAngle(Angle);
    SetLensHeight(LensHeight);
    SetTargetHeight(TargetHeight);
}

inch_t Vision::GetHorizontalDistance() {
	return inch_t(GetDistanceFromTarget()*tan(GetOffsetX()*(std::numbers::pi_v<double>/180.0)));
}

inch_t Vision::GetDistanceFromTargetInch() {
    return inch_t(GetDistanceFromTarget());
}

degree_t Vision::GetOffsetXDeg() {
    return degree_t(GetOffsetX());
}

degree_t Vision::GetOffsetYDeg() {
    return degree_t(GetOffsetX());
}

