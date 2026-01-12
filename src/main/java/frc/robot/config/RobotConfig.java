package frc.robot.config;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Minimal robot configuration reflecting only swerve and vision needs.
 * All superstructure configs (arm/elevator/claw/intake/deploy/climber/lights) are removed.
 */
public record RobotConfig(String robotName, SwerveConfig swerve, VisionConfig vision) {

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      Pose3d leftBackLimelightPosition,
      Pose3d leftFrontLimelightPosition,
      Pose3d rightLimelightPosition,
      Pose3d gamePieceDetectionLimelightPosition) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = RobotBase.isSimulation() ? true : false;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    // Competition-only configuration
    return CompConfig.competitionBot;
  }
}
