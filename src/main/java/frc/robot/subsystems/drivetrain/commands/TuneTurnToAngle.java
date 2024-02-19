package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.IOUtils;

// makes the robot turn to a certain absolute angle (relative to gyro's zero)
public class TuneTurnToAngle extends Command {
        protected Drivetrain drivetrain;  
        protected PIDController pidController;

        private double maxOutput = 1.5*Math.PI; 
    
        public TuneTurnToAngle(Drivetrain drivetrain) {
            // initialize PID controller
            this.pidController = new PIDController(
                Constants.DrivetrainConstants.kTurn_P,
                Constants.DrivetrainConstants.kTurn_I,
                Constants.DrivetrainConstants.kTurn_D
            );
            this.pidController.setTolerance(Constants.DrivetrainConstants.kTurnErrorThreshold);
            this.pidController.enableContinuousInput(-180.0, 180.0);
            
            // DEBUG: add tunable constants to dashboad
            SmartDashboard.putNumber("Angular kP", SmartDashboard.getNumber("Angular kP", Constants.DrivetrainConstants.kTurn_P)); 
            SmartDashboard.putNumber("Angular kI", SmartDashboard.getNumber("Angular kI", Constants.DrivetrainConstants.kTurn_I)); 
            SmartDashboard.putNumber("Angular kD", SmartDashboard.getNumber("Angular kD", Constants.DrivetrainConstants.kTurn_D)); 
            // SmartDashboard.putNumber("Angular kFF", SmartDashboard.getNumber("Angular kFF", Constants.DrivetrainConstants.kTurn_FF)); 
    
            this.drivetrain = drivetrain;

            addRequirements(drivetrain);
        }
    
        @Override
        public void execute() {
            // DEBUG: 
            // System.out.println(SmartDashboard.getNumber("Angular kP", 0.0));
            // this.pidController.setPID(
            //     SmartDashboard.getNumber("Angular kP", 0.0),
            //     SmartDashboard.getNumber("Angular kI", 0.0),
            //     SmartDashboard.getNumber("Angular kD", 0.0)
            // );
            double setpoint = IOUtils.get("angular setpoint", 0);

            this.pidController.setPID(
                SmartDashboard.getNumber("Angular kP", 0.0),
                SmartDashboard.getNumber("Angular kI", 0.0),
                SmartDashboard.getNumber("Angular kD", 0.0)
            );

            // calculate the output angular velocity to run the robot at 
            double output = pidController.calculate(drivetrain.getRotation().getDegrees(), setpoint); 

            // clamp the output so the robot doesn't run TOO fast
            output = MathUtil.clamp(output, -maxOutput, maxOutput); 

            // DEBUG: output the current setpoint, error, etc. to the dashboard
            SmartDashboard.putNumber("setpoint", setpoint); 
            SmartDashboard.putNumber("error", pidController.getPositionError()); 
            // SmartDashboard.putNumber("", output); 
            SmartDashboard.putNumber("output", output); 
            SmartDashboard.putNumber("angle velo", pidController.getVelocityError()); 

            // drive robot with the desired output
            drivetrain.swerveDrive(0, 0, output, false);
        }
    
        @Override
        public void end(boolean interrupted) {}
    
        @Override
        public boolean isFinished() {
            // checks if the robot is at the desired position
            return this.pidController.atSetpoint();
        }

        // set the max rotation speed that the PID controller can give
        public void setMaxOutput(double maxOutput) {
            this.maxOutput = maxOutput; 
        }
}