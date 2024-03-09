package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class SwerveUtils {

    static final double dt = 0.02; 

  // read this: 
  // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/18
  // corrects for swerve drifting while turning by rotating translation vector in the opposite direction
  public static ChassisSpeeds correctInputWithRotation(ChassisSpeeds speeds) {

    // change in angle over a period of dt assuming constant angular velocity
    double angularDisplacement = speeds.omegaRadiansPerSecond * dt;

    // cache the relevant trig
    double sin = Math.sin(0.5 * angularDisplacement);
    double cos = Math.cos(0.5 * angularDisplacement);

    // rotates the direction of output by -theta / 2, theta is the angular speed
    double newVx = speeds.vyMetersPerSecond * sin + speeds.vxMetersPerSecond * cos;
    double newVy = speeds.vyMetersPerSecond * cos - speeds.vxMetersPerSecond * sin;

    return new ChassisSpeeds(newVx, newVy, speeds.omegaRadiansPerSecond); 
  }

  // uses WPILib 2024's discretization to correct inputs for skewing when turning and driving
  public static ChassisSpeeds correctInputWithDiscretization(ChassisSpeeds speeds) {
    Pose2d desiredFuturePose = new Pose2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)); 
    Twist2d twist = new Pose2d().log(desiredFuturePose); 
    return new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt); 
  }

  
  private static final double kEps = 1E-9;

  public static Twist2d alternateLog(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  public static boolean isPoseValid(Pose2d pose) {
      boolean isInLowerXLimit = -2.5 <= pose.getX();
        boolean isInUpperXLimit = pose.getX() <= Constants.Vision.kAprilTagFieldLayout.getFieldLength() + 2.5;
        boolean isInLowerYLimit = -2.5 <= pose.getY();
        boolean isInUpperYLimit = pose.getY() <= Constants.Vision.kAprilTagFieldLayout.getFieldWidth() + 2.5;
        return isInLowerXLimit && isInUpperXLimit && isInLowerYLimit && isInUpperYLimit;
  }
}
