#include <frc/smartdashboard/SmartDashboard.h>
#include "FRC3484_Lib/components/SC_Limelight.h"

#include <frc2/command/SubsystemBase.h>

class VisionSubsystem : public frc2::SubsystemBase {
    
    // Usual Set-up
    public:
        VisionSubsystem(SC::SC_Limelight limeLight);
        GetHeightAngle();
        GetDistance
        
    private:

        SC::SC_Limelight lime_lightV2{50, 10};


};