#ifndef VISION_H
#define VISION_H

#include "FRC3484_Lib/components/SC_Limelight.h"
#include <units/length.h>
#include <units/angle.h>

class Vision: public SC::SC_Limelight {
    public:
        Vision(double Angle, double LensHeight, double TargetHeight);
        units::inch_t GetHorizontalDistance();
        units::inch_t GetDistanceFromTargetInch();
        units::degree_t GetOffsetXDeg();
        units::degree_t GetOffsetYDeg();

    private:
        // std::shared_ptr<nt::NetworkTable> inst;
        // double _targetHeight, _lensHeight
};

#endif