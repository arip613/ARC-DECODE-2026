package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Hood {
	private static final double GEAR_RATIO = 84.0; 

	private final TalonFX motor;
	private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);

	public Hood(TalonFX motor) {
		this.motor = motor;

		var cfg = new TalonFXConfiguration();
		cfg.Slot0 = new Slot0Configs().withKP(80).withKI(0).withKD(0);
		cfg.MotionMagic =
				new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(20.0)
						.withMotionMagicAcceleration(40.0);

		motor.getConfigurator().apply(cfg);
	}

	public void setAngleDegrees(double degrees) {
		double rotations = (degrees / 360.0) * GEAR_RATIO;
		motor.setControl(mmRequest.withPosition(rotations));
	}

	public void hold() {
		double currentRot = motor.getPosition().getValueAsDouble();
		motor.setControl(mmRequest.withPosition(currentRot));
	}
}
