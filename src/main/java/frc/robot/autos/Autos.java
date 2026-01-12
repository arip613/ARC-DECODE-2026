package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

/**
 * Minimal placeholder subsystem for autos. Autonomous is created directly in Robot.
 */
public class Autos extends LifecycleSubsystem {
  public Autos() {
    super(SubsystemPriority.AUTOS);
  }

  public Command getAutoCommand() {
    return Commands.none();
  }
}
