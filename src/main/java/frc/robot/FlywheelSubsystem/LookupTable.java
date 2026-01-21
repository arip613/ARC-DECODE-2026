package frc.robot.FlywheelSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class LookupTable extends StateMachine<LookupTable.State> {
	public enum State { DISABLED, ENABLED }
	public static class ShotPoint {
		public final double distanceMeters;
		public final double rpm;
		public final double hoodangle; 

			public ShotPoint(double distanceMeters, double rpm, double hoodangle) {
			this.distanceMeters = distanceMeters;
			this.rpm = rpm;
			this.hoodangle = hoodangle;
		}
	}

	private final DistanceCalc distanceCalc;
	private final List<ShotPoint> points = new ArrayList<>();
		private final Flywheel flywheel; 
		private final Hood hood; 

		public LookupTable(DistanceCalc distanceCalc, Flywheel flywheel, Hood hood) {
			super(SubsystemPriority.LOCALIZATION, State.DISABLED);
			this.distanceCalc = distanceCalc;
			this.flywheel = flywheel;
			this.hood = hood;

			addPoint(new ShotPoint(1.0, 2000, 10));
			addPoint(new ShotPoint(2.0, 2500, 12));
			addPoint(new ShotPoint(3.0, 3000, 14));
			addPoint(new ShotPoint(4.0, 3500, 16));
			addPoint(new ShotPoint(5.0, 4000, 18));
			addPoint(new ShotPoint(6.0, 4500, 20));
		}

	public void addPoint(ShotPoint point) {
		points.add(point);
		points.sort(Comparator.comparingDouble(p -> p.distanceMeters));
	}

	public double[] lookup(double distanceMeters) {
		if (points.isEmpty()) {
			return new double[] {0.0, 0.0};
		}

		if (distanceMeters <= points.get(0).distanceMeters) {
			return new double[] {points.get(0).rpm, points.get(0).hoodangle};
		}
		if (distanceMeters >= points.get(points.size() - 1).distanceMeters) {
			var last = points.get(points.size() - 1);
			return new double[] {last.rpm, last.hoodangle};
		}

		// essentially its taking the relative avg between two points so if: 
		// 1m = 2000rpm and 2m = 3000rpm then 1.7m = 2700rpm instead of 2500rpm does this make sense
		for (int i = 1; i < points.size(); i++) {
			var low = points.get(i - 1);
			var high = points.get(i);
			if (distanceMeters == low.distanceMeters) {
				return new double[] {low.rpm, low.hoodangle};
			}
			if (distanceMeters == high.distanceMeters) {
				return new double[] {high.rpm, high.hoodangle};
			}
			if (distanceMeters > low.distanceMeters && distanceMeters < high.distanceMeters) {
			double t = (distanceMeters - low.distanceMeters) / (high.distanceMeters - low.distanceMeters);
			var rpm = low.rpm + t * (high.rpm - low.rpm);
			var hood = low.hoodangle + t * (high.hoodangle - low.hoodangle);
			return new double[] {rpm, hood};
			}
		}

		var last = points.get(points.size() - 1);
		return new double[] {last.rpm, last.hoodangle};
	}

		public void enable() { setStateFromRequest(State.ENABLED); }
		public void disable() { setStateFromRequest(State.DISABLED); }

		@Override
		protected State getNextState(State current) { return current; }

		@Override
		public void robotPeriodic() {
			super.robotPeriodic();

			double distance = distanceCalc.getDistanceToAllianceTargetMeters();
			double[] result = lookup(distance);

			// look at me im doing debug stuff
			SmartDashboard.putNumber("Shooter/DistanceMeters", distance);
			SmartDashboard.putNumber("Shooter/TargetRPM", result[0]);
			SmartDashboard.putNumber("Shooter/TargetHoodDeg", result[1]);

			if (getState() == State.ENABLED) {
				if (flywheel != null) {
					flywheel.spinFlywheel(result[0]);
				}
				if (hood != null) {
					hood.setAngleDegrees(result[1]);
				}
			}
		}
}
