# Hang subsystem
## description
This subsystem allows the frc robot to hang on the chain in the stage.


##connected devices
a motor (can sparkmax)


## Methods
| Method Name | Returns | Parameters | Description | Notes |
| --- | --- | ---- | ---- | --- |
|getInstance|instance of hang |N/A|creates a new version of the hang|    |
|hang|void|N/A|once instance is made, creates a constructor for hang|   |
|setupMotors|void|N/A|starts initial code.| |
|setPID|void|p,i,d|determines how "smoothly" the motor for the hang, runs.| |
|periodic|void|N/A|runs codes repeatedly within the method| |
|doSendables|void|N/A|logs data from hang| |
|setManualVelocity|void|velocity|sets the velocity of the motor that we decide| |
|getHangPosition|double|N/A|finds the positon of the hang motor|angle|
|setTargetPosition|void|targetPosition|sets the postion we that want the motor to get to/be at| |
|isAtPosition|void|N/A|determines if the motor is at the needed poistion| |
|getError|double|N/A|determines how far off the motor is from where it should be|current position-target position|
|getHangVelocity|double|N/A|finds the velocity of the motor| |
|getMotorCurrent|double|N/A|find the currunent being outputed in the motor|in amps|
|setHangMode|void|HangControlType type|figures out the hang mode to determine what is happening| |


## what is logged to dashboard
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


## how to use
to use the subsytem, you need to create a mountable motor connected to a sparkmax/encoder


## link to hang subsystem model
for refferences of the hang and how to build it go to this link:
https://docs.wcproducts.com/greyt-telescope