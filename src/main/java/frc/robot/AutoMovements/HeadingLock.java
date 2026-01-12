package frc.robot.AutoMovements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class HeadingLock extends StateMachine<HeadingLock.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;

  // Store targets as points; orientation is irrelevant for heading lock
  private Translation2d redTargetPoint = new Translation2d();
  private Translation2d blueTargetPoint = new Translation2d();
  private double turretOffsetDegrees = 37.0; 

  public enum HeadingLockState {
    DISABLED,
    RED_LOCK,
    BLUE_LOCK;
  }

  public HeadingLock(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.SWERVE, HeadingLockState.DISABLED);
    this.localization = localization;
    this.swerve = swerve;
  }

  // Preferred point-based setters
  public void setRedTargetPoint(Translation2d point) {
    this.redTargetPoint = point;
  }

  public void setBlueTargetPoint(Translation2d point) {
    this.blueTargetPoint = point;
  }

  // Backwards-compatible pose-based setters (ignore rotation)
  public void setRedTargetPose(Pose2d pose) {
    this.redTargetPoint = pose.getTranslation();
  }

  public void setBlueTargetPose(Pose2d pose) {
    this.blueTargetPoint = pose.getTranslation();
  }

  // Accessors for other components that still expect a Pose2d
  public Pose2d getRedTargetPose() {
    return new Pose2d(redTargetPoint, Rotation2d.kZero);
  }

  public Pose2d getBlueTargetPose() {
    return new Pose2d(blueTargetPoint, Rotation2d.kZero);
  }

  // Optional point getters
  public Translation2d getRedTargetPoint() {
    return redTargetPoint;
  }

  public Translation2d getBlueTargetPoint() {
    return blueTargetPoint;
  }

  public void setTurretOffsetDegrees(double offsetDegrees) {
    this.turretOffsetDegrees = offsetDegrees;
  }

  public void enableForAlliance() {
    if (FmsSubsystem.isRedAlliance()) {
      enableRedLock();
    } else {
      enableBlueLock();
    }
  }

  public void enableRedLock() {
    setStateFromRequest(HeadingLockState.RED_LOCK);
  }

  public void enableBlueLock() {
    setStateFromRequest(HeadingLockState.BLUE_LOCK);
  }

  public void disableLock() {
    setStateFromRequest(HeadingLockState.DISABLED);
  }

  @Override
  protected HeadingLockState getNextState(HeadingLockState current) {
    switch (current) {
      case RED_LOCK:
        return HeadingLockState.RED_LOCK;
      case BLUE_LOCK:
        return HeadingLockState.BLUE_LOCK;
      case DISABLED:
      default:
        return HeadingLockState.DISABLED;
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case RED_LOCK -> faceTarget(redTargetPoint);
      case BLUE_LOCK -> faceTarget(blueTargetPoint);
      case DISABLED -> {
      }
    }
  }

  private void faceTarget(Translation2d targetPoint) {
    var robotPose = localization.getPose();
    var dx = targetPoint.getX() - robotPose.getX();
    var dy = targetPoint.getY() - robotPose.getY();
    var angleRadians = Math.atan2(dy, dx); // turret offset handled below
    var angleDegrees = Math.toDegrees(angleRadians);
    swerve.snapsDriveRequest(angleDegrees + turretOffsetDegrees);
  }
}
