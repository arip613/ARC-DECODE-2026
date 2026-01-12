package frc.robot.LockSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class HeadingLock extends StateMachine<HeadingLock.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;

  private Pose2d redTargetPose = new Pose2d();
  private Pose2d blueTargetPose = new Pose2d();
  private double turretOffsetDegrees = 30.0; // tell me the actual offset later dont forget ari im talking to myself

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
    var angleRadians = Math.atan2(dy, dx); //i didnt use swerve gens bc of the offset of the turret and I didnt wanna think abt that
    var angleDegrees = Math.toDegrees(angleRadians);
    swerve.snapsDriveRequest(angleDegrees + turretOffsetDegrees);
  }
}
