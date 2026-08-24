package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.AutoMovements.OutpostSetpoint;
import frc.robot.FlywheelSubsystem.DistanceCalc;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.autos.PointToPointAutos;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.fms.FmsSubsystem;
import frc.robot.currentPhase.phaseTimer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;



public class Robot extends TimedRobot {
  private static final boolean ENABLE_DASHBOARD = true;
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);

  private final VisionSubsystem vision = new VisionSubsystem(
      imu,
      () -> swerve.getDrivetrainState().Pose.getRotation().getDegrees(),
      hardware.leftLimelight, hardware.rightLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final HeadingLock headingLock = new HeadingLock(localization, swerve);
  private final DistanceCalc distanceCalc = new DistanceCalc(localization, headingLock);
  private final Drum drum = new Drum(
      hardware.drumA1,
      hardware.drumA2,
      hardware.drumA3,
      hardware.drumA4);
  private final Hood hood = new Hood(hardware.hoodMotor);
  private final LookupTable turretLookup = new LookupTable(distanceCalc, drum, hood);
  private final intaker intakeRoller = new intaker(
      hardware.intakeRollerMotorA,
      hardware.intakeRollerMotorB);
  private final IntakePosition intakePosition = new IntakePosition(hardware.intakePivotMotor);
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, swerve, intakePosition, intakeRoller);
  private final DrumStateMachine drumSM = new DrumStateMachine(drum);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor, hardware.indexerMotor2);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);



  private final FollowPath.Builder blinePathBuilder;
  private final phaseTimer phaseTimer = new phaseTimer();
  private final PointToPointAutos pointToPointAutos;
  private boolean prevOperatorX = false;
  private boolean prevOperatorB = false;
  private boolean prevOperatorA = false;
  private final Orchestra orchestra = new Orchestra();
  private final Field2d field2d = new Field2d();
  private final edu.wpi.first.math.controller.PIDController trenchYController =
      new edu.wpi.first.math.controller.PIDController(3.0, 0.0, 0.0);
  private Translation2d savedRedTarget;
  private Translation2d savedBlueTarget;
  private Translation2d activePassTarget;
  private boolean rtShootMode = true;
  private static final double SHOOT_SPEED_THRESHOLD = 0.5; // m/s — don't feed if moving faster
  private phaseTimer.Phase lastPhase = null;
  private boolean warningRumbleSent = false;
  // Rumble pattern: array of {duration, pause, duration, pause, ...} in seconds
  // Negative values = rumble off (pause), positive = rumble on
  private double[] rumblePattern = null;
  private int rumblePatternIndex = 0;
  private double rumbleStepEndTime = 0;
  private int shootReadyFrames = 0;
  private static final int SHOOT_READY_FRAME_THRESHOLD = 2;

  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);

    LifecycleSubsystemManager.ready();

    SmartDashboard.putData("Field", field2d);

  orchestra.addInstrument(hardware.drumA1);
  orchestra.addInstrument(hardware.drumA2);
  orchestra.addInstrument(hardware.drumA3);
  orchestra.addInstrument(hardware.drumA4);
    orchestra.addInstrument(hardware.hopperMotor);
    orchestra.addInstrument(hardware.hoodMotor);
  orchestra.addInstrument(hardware.indexerMotor);
  orchestra.addInstrument(hardware.indexerMotor2);
    orchestra.addInstrument(hardware.intakePivotMotor);
  orchestra.addInstrument(hardware.intakeRollerMotorA);
  orchestra.addInstrument(hardware.intakeRollerMotorB);
    orchestra.loadMusic("output.chrp");

    headingLock.setRedTargetPoint(FieldPoints.getHeadingLockRedPoint());
    headingLock.setBlueTargetPoint(FieldPoints.getHeadingLockBluePoint());


    registerNamedCommands();

    try {
      var ppConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        () -> localization.getPose(),                // Pose supplier
        (pose) -> localization.resetPose(pose),      // Pose reset
        () -> swerve.getRobotRelativeSpeeds(),       // Robot-relative speeds supplier
        (speeds, feedforwards) -> swerve.setRobotRelativeAutoSpeeds(speeds), // Drive robot-relative
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),           // Translation PID
          new PIDConstants(5.0, 0.0, 0.0)            // Rotation PID
        ),
        ppConfig,
        () -> FmsSubsystem.isRedAlliance(),          // Flip for red alliance
        swerve                                       // Drive subsystem requirement
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    }

    // BLine-Lib global constraints (no GUI/JSON needed)
    Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
        4.5,   // max velocity m/s
        12.0,  // max acceleration m/s²
        540.0, // max rotational velocity deg/s
        860.0, // max rotational acceleration deg/s²
        0.03,  // end translation tolerance m
        2.0,   // end rotation tolerance deg
        0.2    // intermediate handoff radius m
    ));

    // BLine-Lib path follower setup
    // No withDefaultShouldFlip() — AutoRoutine handles red/blue mirroring itself
  blinePathBuilder = new FollowPath.Builder(
    swerve,                                        // Subsystem requirement
    () -> localization.getPose(),                  // Pose supplier
    () -> swerve.getRobotRelativeSpeeds(),         // ChassisSpeeds supplier
    (speeds) -> swerve.setRobotRelativeAutoSpeeds(speeds), // Drive consumer
    new PIDController(5.0, 0.0, 0.0),             // Translation PID
    new PIDController(5.0, 0.0, 0.0),             // Rotation PID
    new PIDController(2.0, 0.0, 0.0)              // Cross-track PID
  );

    configureBindings();

    // Set up point-to-point auto chooser (shows on SmartDashboard as "Auto Chooser")
  pointToPointAutos = new PointToPointAutos(
    swerve, localization, blinePathBuilder, drum, drumSM, hoodSM,
    headingLock, turretLookup, indexer, hopper, intakeRoller, intakePosition);

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    field2d.setRobotPose(localization.getPose());
    FieldPoints.publishHeadingLockPoints();
    

    // Publish shooter pose (robot-relative offset transformed to field coordinates)
    var shooterField = localization.getPose().transformBy(
        new Transform2d(FieldPoints.SHOOTER_POSE.getTranslation(), FieldPoints.SHOOTER_POSE.getRotation()));
    SmartDashboard.putNumberArray("Shooter/Pose",
        new double[]{shooterField.getX(), shooterField.getY(), shooterField.getRotation().getDegrees()});
    field2d.getObject("Shooter").setPose(shooterField);

  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Use the point-to-point auto chooser from SmartDashboard
    autonomousCommand = pointToPointAutos.getSelected();
    CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Cancel all auto commands but keep the current pose
    CommandScheduler.getInstance().cancelAll();
    // Stop all mechanisms
  drum.stop();
    hood.stopMotor();
    indexer.stop();
    hopper.stop();
    intakeRoller.stop();
    headingLock.disableLock();
    turretLookup.disable();
  drumSM.requestOff();
    hoodSM.requestOff();

    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
    lastPhase = phaseTimer.getCurrentPhase();
    warningRumbleSent = false;
  }

  @Override
  public void teleopPeriodic() {
    phaseTimer.Phase currentPhase = phaseTimer.getCurrentPhase();
    double remaining = phaseTimer.getSecondsRemainingInCurrentPhase();

    // 5 seconds before shift ends: three quick rumble pulses
    if (remaining <= 5.0 && remaining > 4.5 && !warningRumbleSent && rumblePattern == null) {
      // Pattern: on 0.15s, off 0.1s, on 0.15s, off 0.1s, on 0.15s
      rumblePattern = new double[]{0.15, 0.1, 0.15, 0.1, 0.15};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = true;
    }

    // Shift change: one long rumble
    if (lastPhase != null && currentPhase != lastPhase) {
      rumblePattern = new double[]{0.8};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = false;
    }
    lastPhase = currentPhase;

    // Drive the rumble pattern
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (rumblePattern != null) {
      if (rumbleStepEndTime == 0) {
        // Start current step
        boolean isOn = (rumblePatternIndex % 2) == 0;
        hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, isOn ? 1.0 : 0.0);
        rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
      } else if (now >= rumbleStepEndTime) {
        rumblePatternIndex++;
        if (rumblePatternIndex >= rumblePattern.length) {
          // Pattern done
          hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          rumblePattern = null;
        } else {
          boolean isOn = (rumblePatternIndex % 2) == 0;
          hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, isOn ? 1.0 : 0.0);
          rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
        }
      }
    }

    // Operator X/B: adjust shooting angle override
    // NOTE: change behavior to set a fixed one-step override on rising edge so
    // a single X (or B) press moves the aim immediately regardless of prior presses.
    XboxController opXbox = (XboxController) hardware.operatorController.getHID();
    boolean xPressed = opXbox.getXButton();
    boolean bPressed = opXbox.getBButton();
    if (bPressed && !prevOperatorB) {
      // Move a single step to the right (absolute step from center)
      headingLock.setOperatorOverrideDeg(-1.5);
    }
    if (xPressed && !prevOperatorX) {
      // Move a single step to the left (absolute step from center)
      headingLock.setOperatorOverrideDeg(1.5);
    }
    boolean aPressed = opXbox.getAButton();
    if (aPressed && !prevOperatorA) {
      headingLock.setOperatorOverrideDeg(0.0);
    }
    prevOperatorA = aPressed;
    prevOperatorB = bPressed;
    prevOperatorX = xPressed;

    // Phase telemetry
    if (ENABLE_DASHBOARD) {
      SmartDashboard.putString("Phase/Current", currentPhase.name());
      SmartDashboard.putNumber("Phase/ElapsedSec", phaseTimer.getElapsedSec());
      SmartDashboard.putNumber("Phase/SecsInPhase", phaseTimer.getSecondsIntoCurrentPhase());
      SmartDashboard.putNumber("Phase/SecsRemaining", phaseTimer.getSecondsRemainingInCurrentPhase());
    }
  }

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

  private void registerNamedCommands() {
    // Indexer states
    NamedCommands.registerCommand("IndexerOff", Commands.runOnce(() -> indexer.stop()));
    NamedCommands.registerCommand("IndexerIntake", Commands.runOnce(() -> indexer.intake()));
    NamedCommands.registerCommand("IndexerFeed", Commands.runOnce(() -> indexer.feed()));
    NamedCommands.registerCommand("IndexerReverse", Commands.runOnce(() -> indexer.reverse()));

    // Hopper states
    NamedCommands.registerCommand("HopperOff", Commands.runOnce(() -> hopper.stop()));
    NamedCommands.registerCommand("HopperIntake", Commands.runOnce(() -> hopper.intake()));
    NamedCommands.registerCommand("HopperFeed", Commands.runOnce(() -> hopper.feed()));
    NamedCommands.registerCommand("HopperReverse", Commands.runOnce(() -> hopper.reverse()));

    // Intaker (roller) states
    NamedCommands.registerCommand("IntakerOff", Commands.runOnce(() -> intakeRoller.stop()));
    NamedCommands.registerCommand("IntakerIntake", Commands.runOnce(() -> intakeRoller.intake()));
    NamedCommands.registerCommand("IntakerFeed", Commands.runOnce(() -> intakeRoller.feed()));
    NamedCommands.registerCommand("IntakerReverse", Commands.runOnce(() -> intakeRoller.reverse()));

    // Intake position states
    NamedCommands.registerCommand("IntakePositionDeploy", Commands.runOnce(() -> intakePosition.deploy()));
    NamedCommands.registerCommand("IntakePositionRetract", Commands.runOnce(() -> intakePosition.retract()));

  // Drum states
  NamedCommands.registerCommand("ShooterOff", Commands.runOnce(() -> drumSM.requestOff()));
  NamedCommands.registerCommand("ShooterSpin", Commands.runOnce(() -> drumSM.requestRpm(3200.0)));

    // Hood states
    NamedCommands.registerCommand("HoodOff", Commands.runOnce(() -> hoodSM.requestOff()));

    // Heading lock + lookup table: face target and spin up
    NamedCommands.registerCommand("FaceTarget", Commands.runOnce(() -> {
      headingLock.enableForAlliance();
      turretLookup.enable();
    }));
    NamedCommands.registerCommand("FaceTargetOff", Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
  drumSM.requestOff();
      hoodSM.requestOff();
    }));
  }

  private void configureBindings() {
    hardware.driverController.back().onTrue(
      Commands.runOnce(() -> {
        double heading = FmsSubsystem.isRedAlliance() ? 180.0 : 0.0;
        localization.resetGyro(Rotation2d.fromDegrees(heading));
      })
    );


    hardware.operatorController.y().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          drum.dutyCycle(1);
  
        },
        () -> {
         drum.dutyCycle(0);
    
        }
      )
    );
  
      hardware.operatorController.leftTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          drum.dutyCycle(0.5);
  
        },
        () -> {
          drum.stop();
    
        }
      )
    );

    

    hardware.driverController.rightBumper().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          intakePosition.pulse();
  
        },
        () -> {
intakePosition.deploy();    
        }
      )
    );
    
    hardware.driverController.leftBumper().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          intakeRoller.reverse();
  
        },
        () -> {
         intakeRoller.stop();
    
        }
      )
    );

    hardware.driverController.povUp().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          intakePosition.retract();
  
        },
        () -> {
          intakePosition.deploy();
        }
      )
    );


    

    hardware.driverController.leftTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          
        intakePosition.deploy();
        intakeRoller.intake();
        hopper.setDutyPercent(0.6);

  
        },
        () -> {
              

        intakeRoller.stop();
        hopper.stop();
    
        }
      )
    );


   hardware.driverController.x().whileTrue(
      outpost.travelToOutpost()
    );

    // Y button: quick pass at 2100 RPM
    hardware.driverController.y().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          drum.spinDrum(2100);
          hood.setAngleDegrees(-40);
          hopper.feed();
          indexer.feed();
        },
        () -> {
          drum.stop();
          hood.setAngleDegrees(0);
          hopper.stop();
          indexer.stop();
        }
      )
    );



  
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




    hardware.driverController.rightTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          double robotX = localization.getPose().getX();
          boolean shooting = FieldPoints.isInShootZone(robotX);
          rtShootMode = shooting;
          if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/RT_ShootMode", shooting);
          shootReadyFrames = 0;
          if (shooting) {
            turretLookup.enable();
            headingLock.enableForAlliance();
            intakeRoller.intake();
            hopper.feed();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/ShootingActive", true);
          } else {
            double passHeading = FmsSubsystem.isRedAlliance() ? 0.0 : 180.0;
            swerve.snapsDriveRequest(passHeading);
            hopper.feed();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/PassingActive", true);
          }
        },
        () -> {
          boolean wasShootMode = rtShootMode;

          if (wasShootMode) {
            turretLookup.disable();
            headingLock.disableLock();
            drumSM.requestOff();
            hoodSM.requestOff();
            hood.setAngleDegrees(0);
            intakePosition.deploy();
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/ShootingActive", false);
          } else {
            swerve.normalDriveRequest();
            drum.stop();
            hood.setAngleDegrees(0);
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            intakePosition.deploy();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/PassingActive", false);
          }
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          boolean shootMode = rtShootMode;

          if (shootMode) {
            var params = turretLookup.getParameters();
            double rpm = params.flywheelRpm();
            double hoodRad = params.hoodAngleRad();
            drumSM.requestRpm(rpm);
            hoodSM.requestDegrees(Math.toDegrees(hoodRad));
            var speeds = swerve.getRobotRelativeSpeeds();
            double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            boolean slowEnough = robotSpeed < SHOOT_SPEED_THRESHOLD;
            boolean allReady = params.isValid() && slowEnough && drum.isAtGoal() && headingLock.isSettled();
            if (allReady) {
              shootReadyFrames++;
            } else {
              shootReadyFrames = 0;
            }
            if (ENABLE_DASHBOARD) {
              SmartDashboard.putNumber("Driver/RobotSpeed", robotSpeed);
              SmartDashboard.putBoolean("Driver/SlowEnoughToShoot", slowEnough);
              SmartDashboard.putNumber("Driver/ShootReadyFrames", shootReadyFrames);
            }
            if (shootReadyFrames >= SHOOT_READY_FRAME_THRESHOLD) {
              indexer.feed();
              hopper.feed();
            } else {
              indexer.stop();
              hopper.stop();
            }
          } else {
            double passHeading = FmsSubsystem.isRedAlliance() ? 0.0 : 180.0;
            //
            swerve.snapsDriveRequest(passHeading);
            drum.spinDrum(3500);
            hood.setAngleDegrees(-40);

            double currentHeading = localization.getPose().getRotation().getDegrees();
            double headingError = passHeading - currentHeading;
            headingError = ((headingError + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
            boolean headingGood = Math.abs(headingError) <= 5.0;

            if (ENABLE_DASHBOARD) {
              SmartDashboard.putNumber("Pass/RPM", 2500);
              SmartDashboard.putNumber("Pass/HeadingError", headingError);
              SmartDashboard.putBoolean("Pass/HeadingGood", headingGood);
            }
            if (headingGood) {
              indexer.feed();
              hopper.feed();
            } else {
              indexer.stop();
              hopper.stop();
            }
          }
        })
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.sequence(
          edu.wpi.first.wpilibj2.command.Commands.waitSeconds(1.75),
          edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> intakePosition.shooter())
        )
      )
    );


  
  }
    
}