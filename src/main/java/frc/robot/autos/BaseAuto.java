package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class BaseAuto {
  protected final Trailblazer trailblazer;
  private final String autoName;
  private final Command autoCommand;

  protected BaseAuto(Trailblazer trailblazer) {
    this.trailblazer = trailblazer;

    var className = this.getClass().getSimpleName();
    autoName = className.substring(className.lastIndexOf('.') + 1);

    autoCommand = createFullAutoCommand();
  }

  protected abstract Pose2d getStartingPose();

  protected abstract Command createAutoCommand();

  /** Returns the name of this auto. */
  public String name() {
    return autoName;
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  private Command createFullAutoCommand() {
    TrailblazerPathLogger.markAuto(this);
    return createAutoCommand().withName(autoName + "Command");
  }
}
