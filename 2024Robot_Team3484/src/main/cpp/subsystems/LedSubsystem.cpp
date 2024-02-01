//
// LED Subsystem
//
// -Noah
//

#include <subsystems/LedSubsystem.h>

using namespace LedConstants;

LedSubsystem::LedSubsystem() {
    _led_strip.SetLength(LedConstants::LED_STRIP_LENGTH);
    _led_strip.SetData(_led_buffer);
    _led_strip.Start();
}

void LedSubsystem::Periodic() {
    // Runs every 20ms

    switch (_led_state) {
        case solid:
            break;

        case blink:
            if (_blink_timer.Get().value() > _blink_duration) {
                SetSolidState(_solid_r, _solid_g, _solid_b);
                _blink_timer.Stop();

            } else {
                if (int(_blink_timer.Get().value() / _blink_period) % 2 > 0) {
                    _SetAll(_r, _g, _b);

                } else {
                    _SetAll(0, 0, 0);

                }
            }
            break;

        case launch:
            break;
    }
}

void LedSubsystem::_SetAll(int r, int g, int b) {
    for (int i = 0; i < LedConstants::LED_STRIP_LENGTH; i++) {
        _led_buffer[i].SetRGB(r, g, b);
    }

    _led_strip.SetData(_led_buffer);
}

void LedSubsystem::SetSolidState(int r, int g, int b) {
    _SetAll(r, g, b);

    _solid_r = r;
    _solid_g = g;
    _solid_b = b;

    _led_state = solid;
}

void LedSubsystem::SetBlinkState(int r, int g, int b, double duration, double period) {
    _r = r;
    _g = g;
    _b = b;
    _blink_duration = duration;
    _blink_period = period;
    _led_state = blink;

    _blink_timer.Reset();
    _blink_timer.Start();

}

void LedSubsystem::DoLaunch() { // TODO: Finish this when the LEDs are actually attached to the robot
    SetSolidState(_solid_r, _solid_g, _solid_b); 
}

void LedSubsystem::TurnOffLeds() {
    SetSolidState(0, 0, 0);
}