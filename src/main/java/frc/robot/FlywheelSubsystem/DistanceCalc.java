package frc.robot.FlywheelSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;


public class DistanceCalc extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final HeadingLock headingLock;

  public DistanceCalc(LocalizationSubsystem localization, HeadingLock headingLock) {
    super(SubsystemPriority.LOCALIZATION);
    this.localization = localization;
    this.headingLock = headingLock;
  }


  public double getDistanceToAllianceTargetMeters() {
    Pose2d current = localization.getPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();
    return current.getTranslation().getDistance(target.getTranslation());
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
}
