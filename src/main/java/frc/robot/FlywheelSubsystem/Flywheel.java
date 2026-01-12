package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Flywheel {
	private final TalonFX a1, a2, b1, b2;

	public Flywheel(TalonFX a1, TalonFX a2, TalonFX b1, TalonFX b2) {
		this.a1 = a1;
		this.a2 = a2;
		this.b1 = b1;
		this.b2 = b2;
	}

	public void spinFlywheel(double rpm) {
		double rps = rpm / 60.0; // rot p/s i think 
		a1.setControl(new VelocityVoltage(rps));
		a2.setControl(new VelocityVoltage(rps));
		b1.setControl(new VelocityVoltage(-rps));
		b2.setControl(new VelocityVoltage(-rps));
	}

	public void stop() {
		spinFlywheel(0.0);
	}
}
