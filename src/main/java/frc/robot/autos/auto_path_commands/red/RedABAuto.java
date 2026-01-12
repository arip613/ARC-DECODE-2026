package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;

/**
 * Minimal red auto: drive from point A to point B.
 */
public class RedABAuto extends BaseAuto {
  public RedABAuto(Trailblazer trailblazer) {
    super(trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1_FORWARD.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    var start = Points.START_R1_AND_B1_FORWARD.redPose;
    var end = new Pose2d(15.0, start.getY(), Rotation2d.kZero);
    var constraints = new AutoConstraintOptions();

    return trailblazer.followSegment(
        new AutoSegment(constraints, new AutoPoint(start), new AutoPoint(end)));
  }
}
