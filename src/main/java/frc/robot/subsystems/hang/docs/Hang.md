# Hang subsystem
## Description
This subsystem allows the frc robot to hang on the chain in the stage.

## Connected Devices
a motor (can sparkmax)

## Methods
| Method Name | Returns | Parameters | Description | Notes |
| --- | --- | ---- | ---- | --- |
|getInstance|instance of hang |N/A|creates a new version of the hang| can only have one instance a time|
|setupMotors|void|N/A|starts initial code (sets if the motor is inverted, motor rotation limit, position and velocity inversion factors for encoders, determine if in idle, pid values, and burns flash| |
|setPID|void|p,i,d (all return doubles)|determines how "smoothly," and acuratly the motor for the hang, runs. p is proportional wich is a proportional power output depending on error(distance from set point_), i is inergral which is a proportion of error to time, and d is dampening to slow power movement when near setpoint (this allows less harm to the motor and minimizes jiter from p| |
|setManualVelocity|void|velocity|sets the velocity of the motor that we decide| |
|getHangPosition|double|N/A|finds the positon of the hang motor|through rotations|
|setTargetPosition|void|targetPosition|sets the postion we that want the motor to get to/be at| |
|isAtPosition|void|N/A|determines if the motor is at the needed poistion| |
|getError|double|N/A|determines how far off the motor is from where it should be|target position-current position|
|getHangVelocity|double|N/A|finds the velocity of the motor| |
|getMotorCurrent|double|N/A|find the currunent being outputed in the motor|in amps|
|setHangMode|void|HangControlType type|figures out the hang mode to determine how the motor will move|uses a enum class. the modes either run at a constant voltage (MANUAL mode) to test how the arm runs structualy, or the motor goes to a postion using PID (PID mode) for actual use within competition |

## What is Logged To Dashboard
| name | Description | Notes |
| --- | --- | ---- |
|kMotorInverted|logs if the motor is inverted|under setupMotors method|
|kHangMotorLimit|logs the positon limit of the motor|under setupMotors method|
|kPositionConversionFactor|logs the conversion factor of the motor position for the encoders|under setupMotors method|
|kVelocityConversionFactor|logs the conversion factor of the motor velocity for the encoders|under setupMotors method|
|kBrake|logs if the motors are braked(wont move) for idle mode|under setupMotors method|
|Kp, Ki, Kd| logs the p,i,and d for motor soothness|under setupMotors method|
| --- | --- | ---- |
|Hang Current Velocity|logs the velocity of the motor|under doSendables method|
|Hang Current Position|logs the position of the motor|under doSendables method|
|Hang Target Position|logs the postion that is wanted for the motor to go to|under doSendables method|
|Hang Error|logs the error of the motor (distance from wanted position)|under doSendables method|
|Hang Current|logs the current of the motor|under doSendables method|
|Hang Bus Voltage|logs the voltage of the canbus|under doSendables method|
|Hang Output|logs the power output of the spark max|under doSendables method|
| --- | --- | ---- |
|kHangTolerance|logs the tolerance (the allowed error) of the motor|under isAtPosition method|

# How To Use
to use the subsytem, you need to create a mountable motor connected to a sparkmax/encoder
you would also need to have the frc dashboard to be able to tweak and modify code for desired function.

## Methods Needed For The Two Modes
both Manual and Pid mode use simmialr most of the same code since the only difference between the two is that 
manual is for testing purposes and does not use pid for control so that we can see how structualy sound the arm is.
Pid control is primarily for in competition purposes to make the movement accurate, smooth, and to not damage the hang or motor.
#### for instance for Manual and Pid mode, you would have to the "setupMotors"
#### Code For "setupMotors" Method in Pid Mode:
 //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        
        //Sets Smart Limits
        m_HangMotor.setSmartCurrentLimit(20, Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for encoders
        m_HangEncoder.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoder.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotor.setIdleMode(IdleMode.kBrake);

        setPID(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);
        
        m_HangMotor.burnFlash();
notice how we initialize/log all factors for controllability of the motor.
#### code for "setupMotors" method in Manual mode:
 //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        
        //Sets Smart Limits
        m_HangMotor.setSmartCurrentLimit(20, Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for encoders
        m_HangEncoder.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoder.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotor.setIdleMode(IdleMode.kBrake);
        
        m_HangMotor.burnFlash();
now here, notice how we remove the code that logs the "KP," "KI," and "kD," (these are variables for pid) within manual mode. we dont need to worry about not damaging the subsytem or how accurate it is, we just need to be able to move the motor in some way. the whole point of the manual mode is to see how stable the subsystem is and if it breaks or cant move, we know something is wrong and to fix it.
#### and so the only difference between the code within the two modes is that Manual mode does not need to have control with Kp,Ki,Kd and can completely exlude it.
#### with Pid mode we must have all the same methods as manual mode so we can actualy move the arms, as well as include Pid, related code to actualy make the arm precise and efficient.

## link to hang subsystem model 
for references of the hang and how to build it go to this link:
https://docs.wcproducts.com/greyt-telescope