package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;

public class MeteorPattern extends LEDPattern{
     private final Color color;
     private final int PatternLegnth = 5;

    public MeteorPattern(Color color) {
        this(color, 1); 
    }

    public MeteorPattern(Color color, double time) {
        super(time);
        this.color = color;
    }

    @Override
    protected void updateLEDs(AddressableLed buf, double time) {
        for (int i = 0; i < buf.getLength(); i++) {
            for (int currentLed = 0; currentLed <buf.getLength(); currentLed++) {
                if (i - currentLed < PatternLegnth && i-currentLed >0) {
                    buf.setLED(i, color);
                }
                else {
                    buf.setLED(i, Color.kBlack);
                }
            }
            
        }
    }
    
}
