package frc.robot.subsystems.leds;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlState;
import frc.robot.subsystems.leds.patterns.LEDPattern;


public class Led extends SubsystemBase {
    
    private static Led instance = new Led();

    public static Led getInstance() {
        return instance; 
    }
    //Create Led, ledstrips, buffer, pattern, and timer
    AddressableLED m_Led;

    private List<LEDStrip> ledStrips;

    private LEDPattern pattern;

    private AddressableLEDBuffer buffer;

    public boolean isMovingToAmp = false; 
    public boolean isMovingToSpeaker = false; 
    public boolean isShooting = false; 
    public boolean isIntaking = false; 
    public boolean isMovingToNote = false; 
    public boolean beamHit = false;
    public boolean autoBegin = false;
    public boolean autoEnd = false; 
    public boolean isHanging = false; 
    //public boolean isShootingToAmp = false;
    //public boolean isShootingToSpeaker = false;

    private LEDStrip armLeft; 
    private LEDStrip armRight;
    private LEDStrip hangLeft;
    private LEDStrip hangRight;





    private Timer timer = new Timer();

    //If port and Length aren't given, take from Constants. Will then run next Led below
    private Led() {
        this(Constants.LEDs.kLedPort, Constants.LEDs.kLedLength); 
    }

    //If Pattern isn't given, will run with default pattern and then run next Led below
    private Led(int port, int length) {
        // default pattern
        this(port, length, Constants.LEDs.Patterns.kDefault); 
    }

    //Tells Leds what to do with given port, legnth, and pattern
    private Led(int port, int length, LEDPattern pattern) {
        //Sets Port
        this.m_Led = new AddressableLED(port);
        //Sets Length to buffer
        this.m_Led.setLength(length);
        //Actually sets length from buffer to Leds
        this.buffer = new AddressableLEDBuffer(length); 

        armLeft = new LEDStrip(buffer, Constants.LEDs.kLed1Start, Constants.LEDs.kLed1End); 
        armRight = new LEDStrip(buffer, Constants.LEDs.kLed2Start, Constants.LEDs.kLed2End); 
        hangRight = new LEDStrip(buffer, Constants.LEDs.kLed3Start, Constants.LEDs.kLed3End); 
        hangLeft = new LEDStrip(buffer, Constants.LEDs.kLed4Start, Constants.LEDs.kLed4End); 

        this.ledStrips = List.of(
            armLeft,
            armRight,
            hangLeft,
            hangRight

        ); 
            
        //Starts timer
        this.timer.start();
        //Starts Leds!
        this.m_Led.start();
        
        //Set pattern
        setPattern(pattern);


        
        
    }
    //Actual method for Setting Patterns
    public void setPattern(LEDPattern pattern) {
        //If  pattern is already current pattern, exit method
        if (this.pattern == pattern) return; 

        //Set Pattern of each strip
        for (LEDStrip strip : this.ledStrips)
            strip.setPattern(pattern);
        this.pattern = pattern; 
        //Restart timer for whatever reason?
        this.timer.restart();
    }

    public void setArmPattern(LEDPattern pattern) {
        armLeft.setPattern(pattern);
        armRight.setPattern(pattern);
    }

    public void setHangPattern(LEDPattern pattern) {
        hangLeft.setPattern(pattern);
        hangRight.setPattern(pattern);
    }
   

    @Override
    public void periodic() {

        LEDPattern decidedHangPattern = LEDPattern.kEmpty; 
        LEDPattern decidedArmPattern = LEDPattern.kEmpty;

    //    lower priorities
        if (DriverStation.isEnabled()) decidedHangPattern = Constants.LEDs.Patterns.kEnabled; 
        if (DriverStation.isDisabled()) decidedHangPattern = Constants.LEDs.Patterns.kDisabled; 
        if (DriverStation.isAutonomousEnabled()) decidedHangPattern = Constants.LEDs.Patterns.kAuto;
       
         if (Arm.getInstance().getArmControlState() == ArmControlState.TRAVEL) decidedArmPattern = Constants.LEDs.Patterns.kArmMoving;
        if (Arm.getInstance().getArmControlState() == ArmControlState.AMP) decidedArmPattern = Constants.LEDs.Patterns.kArmAtAmp;
        if (Arm.getInstance().getArmControlState() == ArmControlState.SPEAKER) decidedArmPattern = Constants.LEDs.Patterns.kArmAtSpeaker;
        if (Arm.getInstance().getArmControlState() == ArmControlState.GROUND) decidedArmPattern = Constants.LEDs.Patterns.kArmAtGround;
        if (Arm.getInstance().getArmControlState() == ArmControlState.CUSTOM) decidedArmPattern = Constants.LEDs.Patterns.kArmCustom;

        if (this.isMovingToAmp || this.isMovingToSpeaker) decidedArmPattern = Constants.LEDs.Patterns.kMoving;
        if (this.isShooting) decidedArmPattern = Constants.LEDs.Patterns.kShootAnywhere;
        if (this.isIntaking) decidedArmPattern = Constants.LEDs.Patterns.kIntake;
        if (this.isMovingToNote) decidedArmPattern = Constants.LEDs.Patterns.kMovingToNote; 
        if (this.isHanging) decidedArmPattern = Constants.LEDs.Patterns.kHangActive;
        if (this.beamHit) decidedArmPattern = Constants.LEDs.Patterns.kBeamHit; 
        setArmPattern(decidedArmPattern);
        setHangPattern(decidedHangPattern);


        //Constantly updates leds with respect to time
        for (int i = 0; i < ledStrips.size(); i++) {
            ledStrips.get(i).update(timer.get());
        }
        //Sets LED output data
        m_Led.setData(buffer);
    }
    //Method to return how many Led Strips there are
    public List<LEDStrip> getLedStrips() {
        return this.ledStrips; 
    }
    //Method to getLED
    public AddressableLED getLED() {
        return this.m_Led;
    }

    public void doSendables() {
        SmartDashboard.putBoolean("Moving To Amp", this.isMovingToAmp);
        SmartDashboard.putBoolean("Moving To Speaker", this.isMovingToSpeaker);
        SmartDashboard.putBoolean("Is Shooting", this.isShooting);
        SmartDashboard.putBoolean("Is Intaking", this.isIntaking);
    }
}
