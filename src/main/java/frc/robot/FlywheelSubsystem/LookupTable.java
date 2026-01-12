package frc.robot.FlywheelSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class LookupTable extends LifecycleSubsystem {
	public static class ShotPoint {
		public final double distanceMeters;
		public final double rpm;
		public final double hoodHeight; // idk what the units are

		public ShotPoint(double distanceMeters, double rpm, double hoodHeight) {
			this.distanceMeters = distanceMeters;
			this.rpm = rpm;
			this.hoodHeight = hoodHeight;
		}
	}

	private final DistanceCalc distanceCalc;
	private final List<ShotPoint> points = new ArrayList<>();

	public LookupTable(DistanceCalc distanceCalc) {
		super(SubsystemPriority.LOCALIZATION);
		this.distanceCalc = distanceCalc;

// random data points bc I cant think of anything else
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
			return new double[] {points.get(0).rpm, points.get(0).hoodHeight};
		}
		if (distanceMeters >= points.get(points.size() - 1).distanceMeters) {
			var last = points.get(points.size() - 1);
			return new double[] {last.rpm, last.hoodHeight};
		}

		for (int i = 1; i < points.size(); i++) {
			var low = points.get(i - 1);
			var high = points.get(i);
			if (distanceMeters == low.distanceMeters) {
				return new double[] {low.rpm, low.hoodHeight};
			}
			if (distanceMeters == high.distanceMeters) {
				return new double[] {high.rpm, high.hoodHeight};
			}
			if (distanceMeters > low.distanceMeters && distanceMeters < high.distanceMeters) {
				var rpm = (low.rpm + high.rpm) / 2.0;
				var hood = (low.hoodHeight + high.hoodHeight) / 2.0;
				return new double[] {rpm, hood};
			}
		}

		var last = points.get(points.size() - 1);
		return new double[] {last.rpm, last.hoodHeight};
	}

	@Override
	public void robotPeriodic() {
		super.robotPeriodic();
		double distance = distanceCalc.getDistanceToAllianceTargetMeters();
		double[] result = lookup(distance);
	}
}
