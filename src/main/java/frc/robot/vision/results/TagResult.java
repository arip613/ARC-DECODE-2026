package frc.robot.vision.results;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Minimal value type representing a single AprilTag vision measurement.
 */
public class TagResult {
  private final Pose2d pose;
  private final double timestampSeconds;
  private final Vector<N3> standardDevs;

  public TagResult(Pose2d pose, double timestampSeconds, Vector<N3> standardDevs) {
    this.pose = pose;
    this.timestampSeconds = timestampSeconds;
    this.standardDevs = standardDevs;
  }

  public Pose2d pose() {
    return pose;
  }

  public double timestamp() {
    return timestampSeconds;
  }

  public Vector<N3> standardDevs() {
    return standardDevs;
  }
}
