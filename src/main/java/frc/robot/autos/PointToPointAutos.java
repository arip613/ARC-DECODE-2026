package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.FlywheelSubsystem.Drum;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.FlywheelSubsystem.DrumStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.lib.BLine.FollowPath;

/**
 * All point-to-point autonomous routines live here.
 *
 * ===== HOW TO ADD A NEW AUTO =====
 * 1. Add a new method below (copy an existing one as a template)
 * 2. Use AutoRoutine.create(swerve, localization, pathBuilder) to start building
 * 3. Chain steps:
 *      .startAt(x, y, deg)            - set starting pose
 *      .driveTo(x, y, deg)            - drive to a field pose
 *      .runOnce(() -> ...)             - instant action
 *      .run(command)                   - run command, wait for it to finish
 *      .runFor(seconds, command)       - run command for N seconds
 *      .waitSeconds(seconds)           - pause
 *      .doWhileDriving(command)        - run in parallel with NEXT driveTo
 *      .build()                        - finalize
 * 4. Register in constructor: chooser.addOption("Name", myNewAuto())
 *
 * ===== HOW TO ADD A POINT TO AN EXISTING AUTO =====
 * Just insert .driveTo(x, y, heading) wherever you want in the chain.
 * Add .runOnce() / .doWhileDriving() around it for actions.
 *
 * ===== BLUE AUTOS =====
 * Blue autos are mirrored from red using AutoRoutine.createMirrored().
 * Only define red poses — blue is automatic.
 */
public class PointToPointAutos {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final FollowPath.Builder pathBuilder;
  private final Drum drum;
  private final DrumStateMachine drumSM;
  private final HoodStateMachine hoodSM;
  private final HeadingLock headingLock;
  private final LookupTable turretLookup;
  private final Indexer indexer;
  private final Hopper hopper;
  private final intaker intakeRoller;
  private final IntakePosition intakePosition;

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public PointToPointAutos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      FollowPath.Builder pathBuilder,
      Drum drum,
      DrumStateMachine drumSM,
      HoodStateMachine hoodSM,
      HeadingLock headingLock,
      LookupTable turretLookup,
      Indexer indexer,
      Hopper hopper,
      intaker intakeRoller,
      IntakePosition intakePosition) {
    this.swerve = swerve;
    this.localization = localization;
    this.pathBuilder = pathBuilder;
    this.drum = drum;
    this.drumSM = drumSM;
    this.hoodSM = hoodSM;
    this.headingLock = headingLock;
    this.turretLookup = turretLookup;
    this.indexer = indexer;
    this.hopper = hopper;
    this.intakeRoller = intakeRoller;
    this.intakePosition = intakePosition;

    // ===== REGISTER ALL AUTOS HERE =====
    chooser.setDefaultOption("Do Nothing", Commands.none());
    chooser.addOption("Red Right", RedRight());
    chooser.addOption("Red Left", RedLeft());
    chooser.addOption("Blue Right", BlueRight());
    chooser.addOption("Blue Left", BlueLeft());
    chooser.addOption("OutPostRed", OutPostRed());
    chooser.addOption("Red Left One Swipe", RedLeftOneSwipe());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** Get the currently selected auto command from the dashboard chooser. */
  public Command getSelected() {
    return chooser.getSelected();
  }

  // =====================================================================
  //  MIRROR HELPERS — shorthand for FieldPoints mirror utilities
  // =====================================================================

  // Mirror helpers now handled by AutoRoutine.createMirrored

  // =====================================================================
  //  HELPER COMMANDS - reusable building blocks for any auto
  // =====================================================================

  private Command startAiming() {
    return Commands.runOnce(() -> {
      headingLock.enableForAlliance();
      turretLookup.enable();
    });
  }

  private Command stopAiming() {
    return Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
      drumSM.requestOff();
      hoodSM.requestOff();
    });
  }

  private Command startFeeding() {
    return Commands.waitUntil(() -> drum.isAtGoal() && headingLock.isSettled())
        .andThen(Commands.runOnce(() -> {
          indexer.feed();
          hopper.feed();
        }))
        .withName("WaitThenFeed");
  }

  private Command stopFeeding() {
    return Commands.runOnce(() -> {
      indexer.stop();
      hopper.stop();
    });
  }

  private Command startIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.pulse();
      intakeRoller.auto();
      hopper.feed();
    });
  }

  private Command stopIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.retract();
      intakeRoller.stop();
      hopper.stop();
    });
  }

  private Command stopAll() {
    return Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
  drumSM.requestOff();
      hoodSM.requestOff();
      indexer.stop();
      hopper.stop();
      intakeRoller.stop();
      intakePosition.retract();
    });
  }

  // =====================================================================
  //  RED AUTOS (source of truth)
  // =====================================================================



  private Command RedLeftOneSwipe() {
    return AutoRoutine.create(swerve, localization, pathBuilder)
    .startAt(12.11, 0.58, 180.0)
      .driveToAll(8.8, 0.58, 270)
      .run(startIntaking())
      .driveToAll(8.8, 3.413, 270, 1.3)
      .driveToAll(10.62, 0.58, 0)
      .run(stopIntaking())
      .driveToAll(14.8, 0.68, 180)
      .run(startAiming())
      .run(startFeeding())
      .waitSeconds(20)
      .run(stopAiming())
      .run(stopFeeding())
      .run(stopAll())
      .build()
      .withName("Red Left One Swipe");

  }

private Command LeftAuto(boolean mirror) {
  var routine = mirror
      ? AutoRoutine.createMirrored(swerve, localization, pathBuilder)
      : AutoRoutine.create(swerve, localization, pathBuilder);
  return routine
      .startAt(12.11, 0.58, 180.0)
      .driveToAll(8.8, 0.58, 270)
      .run(startIntaking())
      .driveToAll(8.8, 3.413, 270, 2)
      .driveToAll(10.62, 0.68, 0)
      .run(stopIntaking())
      .driveToAll(14.8, 0.68, 180)
      .run(startAiming())
      .run(startFeeding())
      .waitSeconds(4)
      .run(stopAiming())
      .run(stopFeeding())
      .driveToAll(8.81, 0.68, 180)
      .run(startIntaking())
      .driveToAll(10, 3.413, 270,2)
      .driveToAll(10, 0.58, 0)
      .run(stopIntaking())
      .driveToAll(14.8, 0.68, 180)
      .run(startAiming())
      .run(startFeeding())
      .waitSeconds(4)
      .run(stopAll())
      .build()
      .withName(mirror ? "Blue Left" : "Red Left");
}

  private Command RightAuto(boolean mirror) {
    var routine = mirror
        ? AutoRoutine.createMirrored(swerve, localization, pathBuilder)
        : AutoRoutine.create(swerve, localization, pathBuilder);
    return routine
        .startAt(12.11, 7.42, 180.0)
        .driveToAll(8.8, 7.42, 90)
        .run(startIntaking())
        .driveToAll(8.8, 4.587, 90,1.4)
        .driveToAll(10.62, 7.42, 0)
        .run(stopIntaking())
        .driveToAll(14.8, 7.42, 180)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(4)
        .run(stopAiming())
        .run(stopFeeding())
        .driveToAll(8.81, 7.42, 180)
        .run(startIntaking())
        .driveToAll(10, 4.587, 90,2)
        .driveToAll(10, 7.42, 0)
        .run(stopIntaking())
        .driveToAll(14.8, 7.42, 180)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(4)
        .run(stopAll())
        .build()
    .withName(mirror ? "Blue Right" : "Red Right");
  }



  private Command OutPostRed() {
    return AutoRoutine.create(swerve, localization, pathBuilder)
        .startAt(12.11, 7.42, 180.0)
        .driveToAll(8.8, 7.42, 90)
        .run(startIntaking())
        .driveToAll(8.8, 4.587, 90, 1.4)
        .driveToAll(10.62, 7.42, 0)
        .run(stopIntaking())
        .driveToAll(16.275 + 0.254 + 0.102, 7.314, 180, 2.7).withTimeout(6)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(20)
        .run(stopAiming())
        .run(stopFeeding())
        .run(stopAll())
        .build()
        .withName("OutPostRed");
  }






  private Command RedRight() { return RightAuto(false); }
  private Command BlueRight() { return RightAuto(true); }


  private Command RedLeft() { return LeftAuto(false); }
  private Command BlueLeft() { return LeftAuto(true); }


}
