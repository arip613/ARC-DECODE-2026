package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.LimelightHelpers;
import frc.robot.vision.results.TagResult;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private static final double MAX_VISION_XY_STD_DEV = 0.12; 
  private static final double MAX_VISION_THETA_STD_DEV = 0.2; 
  private static final int MIN_TAGS_FOR_HEADING = 8;
  private static final double MAX_HEADING_THETA_STD_DEV = 0.05;
  private static final String RIGHT_LIMELIGHT_NAME = "limelight-right";
  

  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final DoubleArrayPublisher botposeBluePub;
  private final DoubleArrayPublisher robotPosePub;
  private boolean updatePoseWithLeftLimelight = false;

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;

    var nt = NetworkTableInstance.getDefault();
    botposeBluePub = nt.getTable("limelight").getDoubleArrayTopic("botpose_blue").publish();
    robotPosePub = nt.getTable("Localization").getDoubleArrayTopic("robot_pose").publish();

  SmartDashboard.putBoolean("Localization/UseLeftLimelightPose", updatePoseWithLeftLimelight);

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo90",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo270",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
    }
  }

  public Pose2d getPose() {
    return swerve.getDrivetrainState().Pose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerve.getFieldRelativeSpeeds();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    updatePoseWithLeftLimelight = false;
    SmartDashboard.putBoolean("Localization/UseLeftLimelightPose", false);

    vision.getRightTagResult().ifPresent(this::ingestTagResult);

    updateHeadingFromVision();

    Pose2d pose = getPose();
    botposeBluePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        0.0, 
        0.0, 
        0.0, 
        pose.getRotation().getDegrees()
    });

    robotPosePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians()
    });

  }

  private void updateHeadingFromVision() {
    var mt1Estimate = FmsSubsystem.isRedAlliance()
        ? LimelightHelpers.getBotPoseEstimate_wpiRed(RIGHT_LIMELIGHT_NAME)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue(RIGHT_LIMELIGHT_NAME);
    if (mt1Estimate == null || mt1Estimate.tagCount < MIN_TAGS_FOR_HEADING) {
      return;
    }

    double targetHeadingDeg = mt1Estimate.pose.getRotation().getDegrees();

    // Fuse heading through the pose estimator instead of hard-resetting the gyro.
    // Use the current XY with large XY std devs so only the heading component
    // meaningfully corrects the estimate.
    Pose2d current = getPose();
    Pose2d headingPose = new Pose2d(
        current.getTranslation(),
        Rotation2d.fromDegrees(targetHeadingDeg));
    var headingStdDevs = edu.wpi.first.math.VecBuilder.fill(
        100.0, 100.0, MAX_HEADING_THETA_STD_DEV);
    swerve.drivetrain.addVisionMeasurement(
        headingPose, Utils.fpgaToCurrentTime(mt1Estimate.timestampSeconds), headingStdDevs);
  }

  private void ingestTagResult(TagResult result) {
    var visionPose = result.pose();

    if (!isStdDevAcceptable(result.standardDevs())) {
      return;
    }

    if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      resetPoseXYOnly(visionPose);
    }

    var poseXYOnly = new Pose2d(
        visionPose.getTranslation(),
        swerve.getDrivetrainState().Pose.getRotation());

    swerve.drivetrain.addVisionMeasurement(
        poseXYOnly, Utils.fpgaToCurrentTime(result.timestamp()), result.standardDevs());
  }

  private boolean isStdDevAcceptable(edu.wpi.first.math.Vector<edu.wpi.first.math.numbers.N3> devs) {
    if (devs == null) {
      return false;
    }
    double xyStd = Math.max(devs.get(0, 0), devs.get(1, 0));
    double thetaStd = devs.get(2, 0);
    return xyStd <= MAX_VISION_XY_STD_DEV && thetaStd <= MAX_VISION_THETA_STD_DEV;
  }

  public void resetGyro(Rotation2d gyroAngle) {
    // Only use CTRE's offset math — do NOT call Pigeon2.setYaw().
    // setYaw is a CAN command that takes 1-2ms to propagate. If resetRotation
    // reads the gyro before setYaw arrives, the offset is computed against the
    // stale value. When setYaw finally propagates, the heading jumps to
    // 2*desired - oldRaw (typically 90° off).
    swerve.drivetrain.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // resetPose sets both position and rotation in one atomic operation.
    // No separate resetRotation needed — it would just be overwritten.
    swerve.drivetrain.resetPose(estimatedPose);
  }

  public void resetPoseXYOnly(Pose2d estimatedPose) {
    swerve.drivetrain.resetPose(
        new Pose2d(estimatedPose.getTranslation(), swerve.getDrivetrainState().Pose.getRotation()));
  }

  public void setUpdatePoseWithLeftLimelight(boolean enabled) {
    updatePoseWithLeftLimelight = enabled;
    SmartDashboard.putBoolean("Localization/UseLeftLimelightPose", updatePoseWithLeftLimelight);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
