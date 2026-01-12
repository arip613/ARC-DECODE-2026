package frc.robot.AutoMovements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.localization.LocalizationSubsystem;

public class LeaveNeutralZone {

	public enum State {
		omwtoLeaveNeutralZone_red,
		omwtoLeaveNeutralZone_blue,
		leftNeutralZone
	}

	private final LocalizationSubsystem localization;
	private final Trailblazer trailblazer;
	private State state = State.leftNeutralZone;

	public LeaveNeutralZone(LocalizationSubsystem localization, Trailblazer trailblazer) {
		this.localization = localization;
		this.trailblazer = trailblazer;
	}

	public State getState() {
		return state;
	}

	// Delegated setters to centralized FieldPoints
	public void setRA1(Pose2d pose) { FieldPoints.setRA1(pose); }
	public void setRB1(Pose2d pose) { FieldPoints.setRB1(pose); }
	public void setRC1(Pose2d pose) { FieldPoints.setRC1(pose); }
	public void setRA2(Pose2d pose) { FieldPoints.setRA2(pose); }
	public void setRB2(Pose2d pose) { FieldPoints.setRB2(pose); }
	public void setRC2(Pose2d pose) { FieldPoints.setRC2(pose); }

	public void setBA1(Pose2d pose) { FieldPoints.setBA1(pose); }
	public void setBB1(Pose2d pose) { FieldPoints.setBB1(pose); }
	public void setBC1(Pose2d pose) { FieldPoints.setBC1(pose); }
	public void setBA2(Pose2d pose) { FieldPoints.setBA2(pose); }
	public void setBB2(Pose2d pose) { FieldPoints.setBB2(pose); }
	public void setBC2(Pose2d pose) { FieldPoints.setBC2(pose); }

	/**
	 * Start leave-neutral route for Red alliance.
	 * Chooses series 1 (C1,B1,A1) if closer to C1 than C2, otherwise series 2 (C2,B2,A2).
	 * Cancels all other commands on start.
	 */
	public Command omwtoLeaveNeutralZone_red() {
		var constraints = new AutoConstraintOptions();

		// Build series 1: C1 -> B1 -> A1
		var segR1_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getRC1()));
		var segR1_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRC1()), new AutoPoint(FieldPoints.getRB1()));
		var segR1_3 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRB1()), new AutoPoint(FieldPoints.getRA1()));
		var series1 = trailblazer.followSegment(segR1_1, true)
				.andThen(trailblazer.followSegment(segR1_2, true))
				.andThen(trailblazer.followSegment(segR1_3, true))
				.withName("omwtoLeaveNeutralZone_red_series1");

		// Build series 2: C2 -> B2 -> A2
		var segR2_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getRC2()));
		var segR2_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRC2()), new AutoPoint(FieldPoints.getRB2()));
		var segR2_3 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getRB2()), new AutoPoint(FieldPoints.getRA2()));
		var series2 = trailblazer.followSegment(segR2_1, true)
				.andThen(trailblazer.followSegment(segR2_2, true))
				.andThen(trailblazer.followSegment(segR2_3, true))
				.withName("omwtoLeaveNeutralZone_red_series2");

		var chooseSeries1 = (java.util.function.BooleanSupplier) () -> {
			var current = localization.getPose();
			return current.getTranslation().getDistance(FieldPoints.getRC1().getTranslation())
					<= current.getTranslation().getDistance(FieldPoints.getRC2().getTranslation());
		};

		return Commands.runOnce(() -> {
					CommandScheduler.getInstance().cancelAll();
					state = State.omwtoLeaveNeutralZone_red;
				})
				.andThen(Commands.either(series1, series2, chooseSeries1))
				.andThen(Commands.runOnce(() -> state = State.leftNeutralZone))
				.withName("LeaveNeutralZone_red");
	}

	/**
	 * Start leave-neutral route for Blue alliance.
	 * Chooses series 1 (C1,B1,A1) if closer to C1 than C2, otherwise series 2 (C2,B2,A2).
	 * Cancels all other commands on start.
	 */
	public Command omwtoLeaveNeutralZone_blue() {
		var constraints = new AutoConstraintOptions();

		// Build series 1: C1 -> B1 -> A1
		var segB1_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getBC1()));
		var segB1_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBC1()), new AutoPoint(FieldPoints.getBB1()));
		var segB1_3 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBB1()), new AutoPoint(FieldPoints.getBA1()));
		var series1 = trailblazer.followSegment(segB1_1, true)
				.andThen(trailblazer.followSegment(segB1_2, true))
				.andThen(trailblazer.followSegment(segB1_3, true))
				.withName("omwtoLeaveNeutralZone_blue_series1");

		// Build series 2: C2 -> B2 -> A2
		var segB2_1 = new AutoSegment(constraints, new AutoPoint(localization::getPose), new AutoPoint(FieldPoints.getBC2()));
		var segB2_2 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBC2()), new AutoPoint(FieldPoints.getBB2()));
		var segB2_3 = new AutoSegment(constraints, new AutoPoint(FieldPoints.getBB2()), new AutoPoint(FieldPoints.getBA2()));
		var series2 = trailblazer.followSegment(segB2_1, true)
				.andThen(trailblazer.followSegment(segB2_2, true))
				.andThen(trailblazer.followSegment(segB2_3, true))
				.withName("omwtoLeaveNeutralZone_blue_series2");

		var chooseSeries1 = (java.util.function.BooleanSupplier) () -> {
			var current = localization.getPose();
			return current.getTranslation().getDistance(FieldPoints.getBC1().getTranslation())
					<= current.getTranslation().getDistance(FieldPoints.getBC2().getTranslation());
		};

		return Commands.runOnce(() -> {
					CommandScheduler.getInstance().cancelAll();
					state = State.omwtoLeaveNeutralZone_blue;
				})
				.andThen(Commands.either(series1, series2, chooseSeries1))
				.andThen(Commands.runOnce(() -> state = State.leftNeutralZone))
				.withName("LeaveNeutralZone_blue");
	}
}
