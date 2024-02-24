package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;


public class TriFlashPattern extends LEDPattern{
    Color color1;
    Color color2;
    Color color3;
    private double maxTime; 
    
    

    public TriFlashPattern(Color color1, Color color2, Color color3) {
        this(color1, color2, color3, 5); 
        
    }
    
    public TriFlashPattern(Color color1, Color color2,Color color3, double time) {
        super(0);
        this.maxTime = time;
        this.color1 = color1;
        this.color2 = color2;
        this.color3 = color3;
    }

     @Override
     
    protected void updateLEDs(AddressableLed buf, double time) {
        Color decidedColor = Color.kAliceBlue;
        if (time< (this.maxTime/3)) {
            decidedColor = color1;
        }

        else if (time > (time/3) && time < 2*this.maxTime/3) {
            decidedColor = color2;
        }
        else if (time >2*this.maxTime/3) {
          decidedColor = color3;
        }
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setLED(i, decidedColor);
            

        }
        
    }


}
    
    

