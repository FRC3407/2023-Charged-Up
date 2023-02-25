package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.team3407.drive.*;
import frc.robot.team3407.drive.Types.*;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.ADIS16470_3X.IMUAxis;


public final class Constants {

	public static final DriveMap_4<WPI_TalonSRX>
		DRIVEBASE_LAYOUT = new DriveMap_4<>(
			5, 2, 4, 3,			// note that the 'front' motors for each side should be the ones that have the encoders plugged in
			Motors.can_talonsrx,
			Inversions.RIGHT,
			DriveLayout.DIFFERENTIAL
		);
	public static final NeutralMode
		DRIVEBASE_NEUTRAL_MODE = NeutralMode.Brake;
	public static final DriveBase.ClosedLoopParams
		DRIVEBASE_PARAMS = new DriveBase.ClosedLoopParams(  // MAKE SURE THESE ARE ALL CORRECT BEFORE TEST DRIVING IT!!!
			0.58204,		// use the 'empirical' value from characterization here rather than the actual width
			0.1524,		// 6 inches, coverted to meters
			0.77652,		// angular is 1.239
			2.1418,			// angular is 2.3416
			0.81806,		// angular is 0.50996
			2.8089,			// angular is 3.0669
			0.0,
			0.0,
			2.0,
			0.7,
			10.0,
			3.0,
			2.5,
			Inversions.BOTH
		);
	public static final int
		ARM_WINCH_CAN_ID = 10,
		ARM_EXTENDER_CAN_ID = 12,
		GRABBER_CAN_ID = 11,
		GRABBER_WRIST_PWM_PORT = 0
	;

	public static final IMUAxis
		IMU_YAW_AXIS = IMUAxis.kZ,
		IMU_PITCH_AXIS = IMUAxis.kY		// on the new robot the IMU is turned sidewasy so pitch is kY, not kX like before
	;

	public static final double
		DRIVE_INPUT_DEADZONE = 0.05,
		DRIVE_INPUT_VEL_SCALE = -3.0,
		DRIVE_INPUT_EXP_POWER = 1.0,

		IMU_RATE_FILTER = 0.35,

		ACTIVE_PARK_VOLTS_PER_METER = 100.0,
		BALANCE_PARK_VOLTS_PER_DEGREE = 0.2,
		AUTO_PAD_ENGAGE_VELOCITY = 0.8,
		AUTO_PAD_INCLINE_VELOCITY = 0.5,	// set to 0.1 if we ever fix ff/fb for inclines

		ARM_ANGLE_KF = 1.0,
		ARM_ANGLE_KP = 1.0,
		ARM_ANGLE_KI = 0.0,
		ARM_ANGLE_KD = 0.0,
		ARM_ANGLE_CRUISE_DEG_PER_SEC = 60.0,	// mike said ~10-15% output for cruising speed
		ARM_ANGLE_ACC_DEG_PER_SEC_SQRD = 120.0,

		GRAB_POSITION_KF = 1.0,
		GRAB_POSITION_KP = 1.0,
		GRAB_POSITION_KI = 0.0,
		GRAB_POSITION_KD = 0.0
	;

	public static final double
		GRABBER_ROT_RADIUS_INCHES = 4.9787,				// the radius of the gear-driven linkage
		GRABBER_FINGER_OFFSET_INCHES = 1.5093,			// offset between linkage pivot and finger "grab surface"
		GRABBER_PIVOT_OFFSET_INCHES = 1.8889,			// offset between large gear pivot and "center" - half the distance between large gear centers
		GRABBER_GEARING_IN2OUT = (28.0 / 12.0),			// Input has 12 teeth, output has 28 teeth - "input rotations per output rotations"
		GRABBER_A0_WIDTH_INCHES = Manipulator.Grabber.grabAngleToWidth(0.0),
		GRABBER_W0_ANGLE_DEGREES = Manipulator.Grabber.grabWidthToAngle(0.0)
	;

	public static final int		// see: https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
		SRX_MAG_UNITS_PER_REVOLUTION = 4096,
		FALCON_UNITS_PER_REVOLUTION = 2048,
		NEVEREST_UNITS_PER_REVOLUTION = (7 * 60 * 4),	// 7 pulse/rot * 60:1 gearing * 4 counts/pulse as counted by TalonSRX
		ANALOG_POT_UNITS_PER_REVOLUTION = 1024
	;

	public static final String
		TEST_TRAJECTORY = "Test Auto";
	public static final HashMap<String, Command> AUTO_EVENTS = new HashMap<>();



	public static class ButtonBox extends InputMap {
		public static enum Digital implements DigitalMap {
			B1(1), B2(2), B3(3), B4(4), B5(5), B6(6),
			S1(7), S2(8), TOTAL(8);

			public final int value;
			private Digital(int v) { this.value = v; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private ButtonBox() {}
		public static final ButtonBox Map = new ButtonBox();

		public boolean compatible(GenericHID i)
			{ return Digital.TOTAL.compatible(i); }
		public boolean compatible(int p)
			{ return Digital.TOTAL.compatible(p); }
	}


}