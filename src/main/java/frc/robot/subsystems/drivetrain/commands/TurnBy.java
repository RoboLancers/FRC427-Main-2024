package frc.robot.subsystems.drivetrain.commands;

import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * @see TurnToAngle
 */
public class TurnBy extends TurnToAngle {

        private double initialAngle; 
        private double additionalAngle; 

        public TurnBy(Drivetrain drivetrain, double angle) {
            // hacky; set setpoint to zero first
            super(drivetrain, 0);
            this.additionalAngle = angle; 
        }

        @Override
        public void initialize() {
            super.initialize();
            // when it's time to run the command, set the setpoint to whatever we need it to be 
            // (in this case, the drivetrain's initial angle + the inputted angle)
            this.initialAngle = this.drivetrain.getRotation().getDegrees(); 
            this.setpoint = this.initialAngle + this.additionalAngle;
        }
}