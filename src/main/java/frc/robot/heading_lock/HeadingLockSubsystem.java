package frc.robot.heading_lock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class HeadingLockSubsystem extends StateMachine<HeadingLockSubsystem.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;

  private Pose2d redTargetPose = new Pose2d();
  private Pose2d blueTargetPose = new Pose2d();

  public enum HeadingLockState {
    DISABLED,
    RED_LOCK,
    BLUE_LOCK;
  }

  public HeadingLockSubsystem(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.SWERVE, HeadingLockState.DISABLED);
    this.localization = localization;
    this.swerve = swerve;
  }

  public void setRedTargetPose(Pose2d pose) {
    this.redTargetPose = pose;
  }

  public void setBlueTargetPose(Pose2d pose) {
    this.blueTargetPose = pose;
  }

  public Pose2d getRedTargetPose() {
    return redTargetPose;
  }

  public Pose2d getBlueTargetPose() {
    return blueTargetPose;
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
      case RED_LOCK -> faceTarget(redTargetPose);
      case BLUE_LOCK -> faceTarget(blueTargetPose);
      case DISABLED -> {
      }
    }
  }

  private void faceTarget(Pose2d target) {
    var robotPose = localization.getPose();
    var dx = target.getX() - robotPose.getX();
    var dy = target.getY() - robotPose.getY();
    var angleRadians = Math.atan2(dy, dx);
    var angleDegrees = Math.toDegrees(angleRadians);
    swerve.snapsDriveRequest(angleDegrees);
  }
}
