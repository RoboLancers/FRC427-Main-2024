# Hang subsystem
## Description
This subsystem allows the frc robot to hang on the chain in the stage.

## Connected Devices
a motor (can sparkmax)

## Methods
| Method Name | Returns | Parameters | Description | Notes |
| --- | --- | ---- | ---- | --- |
|`getInstance`|instance of hang |N/A|creates a new version of the hang| can only have one instance a time|
|`setupMotors`|void|N/A|starts initial code (sets if the motor is inverted, motor rotation limit, position and velocity inversion factors for encoders, determine if in idle, pid values, and burns flash| |
|`setPID`|void|p,i,d (all return doubles)|determines how "smoothly," and acuratly the motor for the hang, runs. p is proportional wich is a proportional power output depending on error(distance from set point_), i is inergral which is a proportion of error to time, and d is dampening to slow power movement when near setpoint (this allows less harm to the motor and minimizes jiter from p| |
|`setManualVelocity`|void|velocity|sets the velocity of the motor that we decide| |
|`getHangPosition`|double|N/A|finds the positon of the hang motor|through rotations|
|`setTargetPosition`|void|targetPosition|sets the postion we that want the motor to get to/be at| |
|`isAtPosition`|void|N/A|determines if the motor is at the needed poistion| |
|`getError`|double|N/A|determines how far off the motor is from where it should be|target position-current position|
|`getHangVelocity`|double|N/A|finds the velocity of the motor| |
|`getMotorCurrent`|double|N/A|find the currunent being outputed in the motor|in amps|
|`setHangMode`|void|HangControlType type|figures out the hang mode to determine how the motor will move|uses a enum class. the modes either run at a constant voltage (MANUAL mode) to test how the arm runs structualy, or the motor goes to a postion using PID (PID mode) for actual use within competition |

## What is Logged To Dashboard
| name | Description | Notes |
| --- | --- | ---- |
|`kMotorInverted`|logs if the motor is inverted|under setupMotors method|
|`kHangMotorLimit`|logs the positon limit of the motor|under setupMotors method|
|`kPositionConversionFactor`|logs the conversion factor of the motor position for the encoders|under setupMotors method|
|`kVelocityConversionFactor`|logs the conversion factor of the motor velocity for the encoders|under setupMotors method|
|`kBrake`|logs if the motors are braked(wont move) for idle mode|under setupMotors method|
|`Kp`, `Ki`, `Kd`| logs the p,i,and d for motor soothness|under setupMotors method|
| --- | --- | ---- |
|`Hang Current Velocity`|logs the velocity of the motor|under doSendables method|
|`Hang Current Position`|logs the position of the motor|under doSendables method|
|`Hang Target Position`|logs the postion that is wanted for the motor to go to|under doSendables method|
|`Hang Error`|logs the error of the motor (distance from wanted position)|under doSendables method|
|`Hang Current`|logs the current of the motor|under doSendables method|
|`Hang Bus Voltage`|logs the voltage of the canbus|under doSendables method|
|`Hang Output`|logs the power output of the spark max|under doSendables method|
| --- | --- | ---- |
|`kHangTolerance`|logs the tolerance (the allowed error) of the motor|under isAtPosition method|

## How To Use
to use the subsytem, you need to create a mountable motor connected to a sparkmax/encoder
you would also need to have the frc dashboard to be able to tweak and modify code for desired function.
### manual mode (why and how to use)
manual mode is important to testing the hang mechanism, 
specifically how stable, and sturcturaly sound the hang is.
to set up you would need to manualy switch the mode in 
```setHangMode``` to the `MANUAL` enum within smart dashboard.
After this, to move the arm, you would also have to set 
the value for the `setManualVelocity` method.
### -----------------------------------------------------------
### Pid mode (why and how to use)
Pid mode is used in competition primarily, the reason is that Pid control is used to smooth the motor movement to not damage the hang mechanism/motor, and to make it more accurate. to set up you would need to manualy switch the mode in ```setHangMode``` to the `PID` enum within smart dashboard. After this, to move the arm, you would also have to set the values for the ```setTargetPosition``` methods. the key to PID mode is to also tune the ```p```, ```i```, ```d``` paremeters under the `setPID` method. the easiest way to tune the pid is to start with ```p```, and work down to ```d```, and repeating. ```p``` is used to actualy power the motor till it reaches the ```SetTargetPosition``` value. in very small amounts, increase the ```p``` till it starts to jitter around where the set point would be. in most to all cases, ```i``` is never used and so for the hang you dont have to worry about tuning it. ```d``` is used to dampen movement around the ```setTargetPosition``` and the motors starting position. very slightly increase d, until there is a little jitter as posibble before repeating.

### saftey precautions
while using the hang subsytem, be carfull of putting your body right above the hang mechanism or under it. make sure that while coding, you tune slowly and with little incriments so you dont make the motor break, or make the mechanism shoot to a position and possibly hurt the robot and a person.

## link To Hang Subsytem Reference
https://docs.wcproducts.com/greyt-telescope