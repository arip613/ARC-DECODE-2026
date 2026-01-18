package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {
// public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public final CommandXboxController driverController = new CommandXboxController(0);
   public final Limelight leftLimelight =
    new Limelight("left", LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight =
    new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);
}
