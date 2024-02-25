package frc.robot.util;


public class ChassisState {
  /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
  public double vxMetersPerSecond;

  /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
  public double vyMetersPerSecond;

  /** Represents the angular velocity of the robot frame. (CCW is +) */
  public double omegaRadians;

  public boolean turn; 

  /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
  public ChassisState() {}

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   */
  public ChassisState(
      double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadians, boolean turn) {
    this.vxMetersPerSecond = vxMetersPerSecond;
    this.vyMetersPerSecond = vyMetersPerSecond;
    this.omegaRadians = omegaRadians;
    this.turn = turn; 
  }

  @Override
  public String toString() {
    return String.format(
        "ChassisState(Vx: %.2f m/s, Vy: %.2f m/s, Theta: %.2f rad, Turn: %s )",
        vxMetersPerSecond, vyMetersPerSecond, omegaRadians, turn);
  }

}
