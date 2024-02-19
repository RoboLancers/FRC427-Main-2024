package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;
import frc.robot.util.quad.OrderedPair;
import frc.robot.util.quad.Quadrilateral;

public class GeneralizedHangRoutine extends Command {
    
    public DriverController driverController;
    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    private double angleToTurn; 
    private boolean stop = false; 

    public GeneralizedHangRoutine(DriverController driverController, Drivetrain drivetrain, Arm arm, Intake intake) {
        this.arm = arm;
        this.driverController = driverController;
        this.intake = intake;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain, arm, intake); 
    }

    public void initialize() {
        this.stop = false; 
        Optional<Double> angle = getAngle(drivetrain.getPose());
        if (angle.isEmpty()) {
            stop = true; 
        } else {
            this.angleToTurn = angle.get(); 
        }
        arm.goToAngle(90);
    }

    public void execute() {
        ChassisState speeds = driverController.getDesiredChassisState(); 
        speeds.omegaRadians = Math.toRadians(this.angleToTurn);
        speeds.turn = true; 

        double newVx = speeds.vxMetersPerSecond * Math.cos(speeds.omegaRadians) - speeds.vyMetersPerSecond * Math.sin(speeds.omegaRadians); 
        double newVy = speeds.vxMetersPerSecond * Math.sin(speeds.omegaRadians) + speeds.vyMetersPerSecond * Math.cos(speeds.omegaRadians); 

        speeds.vxMetersPerSecond = newVx; 
        speeds.vyMetersPerSecond = newVy;

        drivetrain.swerveDriveFieldRel(speeds, false);
    }

    public boolean isFinished() {
        return stop;
    }

    public void end(boolean interrupted) {
        arm.goToAngle(0); 
    }



    
        // basically tbe domainn and range for x and y values of the command
    public static boolean isPoseInRectangle(OrderedPair position, OrderedPair topLeft, OrderedPair bottomRight, OrderedPair topRight, OrderedPair bottomLeft) {
        Quadrilateral quad = new Quadrilateral(topLeft, bottomRight, topRight, bottomLeft); 

        return quad.isPointInterior(position); 
    }

    // write a method that returns an angle based on the position it's in 
    // if the robot is in allaicne red but the robot is in a blue thingy, then just return 0 or somtehing
    public static Optional<Double> getAngle(Pose2d robotPose) {
        Alliance alliance = getTargetPose();

        if (alliance == Alliance.Blue) {
            if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft1,
                Constants.AutoHang.blueBottomRight1,
                Constants.AutoHang.blueTopRight1,
                Constants.AutoHang.blueTopLeft1
            )) {
                return Optional.of(-60.0);
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft2,
                Constants.AutoHang.blueBottomRight2,
                Constants.AutoHang.blueTopRight2,
                Constants.AutoHang.blueTopLeft2
            )) {
                return Optional.of(180.0);
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft3,
                Constants.AutoHang.blueBottomRight3,
                Constants.AutoHang.blueTopRight3,
                Constants.AutoHang.blueTopLeft3
            )) {
                return Optional.of(60.0);
            }
        }
        if (alliance == Alliance.Red) {
            if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft1,
                Constants.AutoHang.redBottomRight1,
                Constants.AutoHang.redTopRight1,
                Constants.AutoHang.redTopLeft1
            )) {
                return Optional.of(-120.0);
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft2,
                Constants.AutoHang.redBottomRight2,
                Constants.AutoHang.redTopRight2,
                Constants.AutoHang.redTopLeft2
            )) {
                return Optional.of(0.0);
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft3,
                Constants.AutoHang.redBottomRight3,
                Constants.AutoHang.redTopRight3,
                Constants.AutoHang.redTopLeft3
            )) {
                return Optional.of(120.0);
            }
        }

        return Optional.empty();
    }

    public static Alliance getTargetPose() {
        Optional<Alliance> optAlliance = DriverStation.getAlliance();

        if (optAlliance.isEmpty()) return null;

        Alliance alliance = optAlliance.get(); 
        return alliance; 
    }
}
