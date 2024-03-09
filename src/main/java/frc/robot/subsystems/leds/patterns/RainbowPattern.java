package frc.robot.subsystems.leds.patterns;

import frc.robot.subsystems.leds.AddressableLed;

public class RainbowPattern extends LEDPattern {

    public RainbowPattern(double rainbowTime) {
        super(rainbowTime);
    }

    @Override
    protected void updateLEDs(AddressableLed buffer, double time) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, (int) ((i + time * buffer.getLength() / getLoopTime() ) * 180 / buffer.getLength()) % 180, 255, 128);
        }
    }
}
