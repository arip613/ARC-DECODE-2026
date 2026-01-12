package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.heading_lock.HeadingLockSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Robot extends TimedRobot {
  // Hard-coded alliance assumption for autonomous start pose; set true for Red, false for Blue.
  private static final boolean ASSUME_RED_ALLIANCE = false;
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);
  private final Limelight leftLimelight =
    new Limelight("left", LimelightState.TAGS, LimelightModel.THREEG);
  private final Limelight rightLimelight =
    new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

  private final VisionSubsystem vision = new VisionSubsystem(imu, leftLimelight, rightLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final HeadingLockSubsystem headingLock = new HeadingLockSubsystem(localization, swerve);

  public Robot() {

    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());



  LifecycleSubsystemManager.ready();

  // Set heading lock targets: Red -> (1, 3, 4°), Blue -> (5, 12, 1°)
  headingLock.setRedTargetPose(new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(4.0)));
  headingLock.setBlueTargetPose(new Pose2d(5.0, 12.0, Rotation2d.fromDegrees(1.0)));

  configureBindings();

  ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    

    // Field calibration features removed in simplified build
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Build a minimal A->B auto using a hard-coded alliance assumption (no FMS check)
    var isRed = ASSUME_RED_ALLIANCE;
    var startPose = isRed ? Points.START_R1_AND_B1_FORWARD.redPose : Points.START_R1_AND_B1_FORWARD.bluePose;
    var endPose = new edu.wpi.first.math.geometry.Pose2d(15.0, startPose.getY(), edu.wpi.first.math.geometry.Rotation2d.kZero);

    // Reset pose before starting
    localization.resetPose(startPose);

    var constraints = new AutoConstraintOptions();
    var segment = new AutoSegment(constraints, new AutoPoint(startPose), new AutoPoint(endPose));

    autonomousCommand = trailblazer
      .followSegment(segment, true)
      .withName((isRed ? "Red" : "Blue") + "_A_to_B_Auto");

  edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  if (DriverStation.isTeleop()) {
                    swerve.driveTeleop(
                        hardware.driverController.getLeftX(),
                        hardware.driverController.getLeftY(),
                        hardware.driverController.getRightX());
                  }
                })
            .withName("DefaultSwerveCommand"));

  // Keep only a minimal binding to zero gyro
  hardware.driverController.back().onTrue(localization.getZeroCommand());

  // Enable heading lock for current alliance on Y press
  hardware.driverController.y().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(headingLock::enableForAlliance));
  }
}
