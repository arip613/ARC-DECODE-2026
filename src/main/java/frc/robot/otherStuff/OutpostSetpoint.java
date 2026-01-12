package frc.robot.otherStuff;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;


public class OutpostSetpoint {
	public enum State {
		omwToOutpost,
		atOutpost
	}

	private final LocalizationSubsystem localization;
	private final Trailblazer trailblazer;
	private State state = State.atOutpost;

	private Pose2d redSetpoint = new Pose2d(9.0, 9.0, Rotation2d.fromDegrees(0.0));
	private Pose2d blueSetpoint = new Pose2d(10.0, 10.0, Rotation2d.fromDegrees(180));

	public OutpostSetpoint(LocalizationSubsystem localization, Trailblazer trailblazer) {
		this.localization = localization;
		this.trailblazer = trailblazer;
	}

	public Pose2d getAllianceSetpoint() {
		return FmsSubsystem.isRedAlliance() ? redSetpoint : blueSetpoint;
	}

	public void setRedSetpoint(Pose2d pose) {
		this.redSetpoint = pose;
	}

	public void setBlueSetpoint(Pose2d pose) {
		this.blueSetpoint = pose;
	}

	public State getState() {
		return state;
	}

	public Command travelToOutpost() {
		var constraints = new AutoConstraintOptions();
		var segment =
				new AutoSegment(
						constraints,
						new AutoPoint(localization::getPose),
						new AutoPoint(this::getAllianceSetpoint));

		return Commands.runOnce(() -> state = State.omwToOutpost)
				.andThen(
						trailblazer
								.followSegment(segment, true)
								.withName("omwToOutpost"))
				.andThen(
						Commands.runOnce(
								() -> {
									state = State.atOutpost;
								}))
				.withName("travelToOutpost");
	}
}
