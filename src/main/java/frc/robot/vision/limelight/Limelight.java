package frc.robot.vision.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.vision.results.OptionalTagResult;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;

/**
 * Simplified Limelight wrapper for a two-camera setup that only performs AprilTag-based
 * odometry updates. All non-tag features (game pieces, handoff, LEDs, calibration) are removed.
 */
public class Limelight extends StateMachine<LimelightState> {
  private static final double IS_OFFLINE_TIMEOUT = 3.0;
  private static final double USE_MT1_DISTANCE_THRESHOLD = Units.inchesToMeters(40.0);

  private final String limelightTableName;
  @SuppressWarnings("unused")
  private final String name;
  @SuppressWarnings("unused")
  private final LimelightModel limelightModel;

  private final Timer limelightTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;

  private double lastTimestamp = 0.0;

  private double angularVelocity = 0.0;

  // Retained for API compatibility with VisionSubsystem's helper
  private final int[] closestScoringReefTag = {0};
  private OptionalTagResult tagResult = new OptionalTagResult();

  public Limelight(String name, LimelightState initialState, LimelightModel limelightModel) {
    super(SubsystemPriority.VISION, initialState);
    this.limelightTableName = "limelight-" + name;
    this.name = name;
    this.limelightModel = limelightModel;
    limelightTimer.start();
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    this.angularVelocity = angularVelocity;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  /**
   * Get tag-based pose estimate suitable for odometry fusion.
   */
  public OptionalTagResult getTagResult() {
    var mT2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);
    if (mT2Estimate == null) {
      updateHealth(tagResult.empty());
      return tagResult.empty();
    }

    if (Math.abs(angularVelocity) > 360) {
      updateHealth(tagResult.empty());
      return tagResult.empty();
    }

    if (mT2Estimate.tagCount == 0) {
      updateHealth(tagResult.empty());
      return tagResult.empty();
    }

    if (mT2Estimate.rawFiducials.length == 1) {
      double ambiguity = mT2Estimate.rawFiducials[0].ambiguity;
      if (ambiguity >= 0.7) {
        updateHealth(tagResult.empty());
        return tagResult.empty();
      }
    }

    if (FeatureFlags.VISION_STALE_DATA_CHECK.getAsBoolean()) {
      var newTimestamp = mT2Estimate.timestampSeconds;
      if (newTimestamp == lastTimestamp) {
        updateHealth(tagResult.empty());
        return tagResult.empty();
      }
      lastTimestamp = newTimestamp;
    }

    var mt2Pose = mT2Estimate.pose;
    // Guard against limelight losing power and reporting zero pose
    if (mt2Pose.getX() == 0.0 && mt2Pose.getY() == 0.0) {
      updateHealth(tagResult.empty());
      return tagResult.empty();
    }

    var devs = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
    if (FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      var distance = mT2Estimate.avgTagDist;

      var xyDev = 0.01 * Math.pow(distance, 1.2);
      var thetaDev = 0.03 * Math.pow(distance, 1.2);

      devs = VecBuilder.fill(xyDev, xyDev, thetaDev);

      // Prefer MT1 rotation when close to tags for better heading
      if (distance <= USE_MT1_DISTANCE_THRESHOLD) {
        var mT1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
        if (mT1Result != null
            && mT1Result.tagCount != 0
            && mT1Result.pose.getRotation().getDegrees() != 0.0) {
          mt2Pose = new Pose2d(mT2Estimate.pose.getTranslation(), mT1Result.pose.getRotation());
        }
      }
    }

    tagResult = tagResult.update(mt2Pose, mT2Estimate.timestampSeconds, devs);
    updateHealth(tagResult);
    return tagResult;
  }

  public void setClosestScoringReefTag(int tagID) {
    closestScoringReefTag[0] = tagID;
  }

  private void updateHealth(OptionalTagResult result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.reset();
      limelightTimer.start();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      return;
    }

    if (result.isPresent()) {
      cameraHealth = CameraHealth.GOOD;
    } else {
      cameraHealth = CameraHealth.NO_TARGETS;
    }
  }

  public CameraHealth getCameraHealth() {
    return cameraHealth;
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, OFF -> getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
  }

}
