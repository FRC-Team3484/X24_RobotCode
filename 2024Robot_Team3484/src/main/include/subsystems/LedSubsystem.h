#ifndef LED_H
#define LED_H


#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/AddressableLED.h>
#include <frc/Timer.h>

#include <Constants.h>

class LedSubsystem : public frc2::SubsystemBase {
    public:
        LedSubsystem(
            int LED_STRIP_LENGTH
        );

        void Periodic() override;
        void SetSolidState(int r, int g, int b);
        void SetBlinkState(int r, int g, int b, double duration, double period);
        void DoLaunch();
        void TurnOffLeds();

    private:
        void _SetAll(int r, int g, int b);

        frc::AddressableLED _led_strip{LedConstants::LED_STRIP_PWM};
        std::array<frc::AddressableLED::LEDData, LedConstants::LED_STRIP_LENGTH> _led_buffer; 
        enum State {solid, blink, launch};
        State _led_state = solid;

        frc::Timer _blink_timer;

        int _r;
        int _g;
        int _b;

        int _solid_r;
        int _solid_g;
        int _solid_b;

        int _blink_duration;
        int _blink_period;
};


#endif