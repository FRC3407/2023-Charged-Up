package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.team3407.drive.*;
import frc.robot.team3407.drive.Types.*;


public final class Constants {

    public static final DriveMap_4<WPI_TalonSRX>
        DRIVEBASE_LAYOUT = new DriveMap_4<>(
            5, 2, 4, 3,			// note that the 'front' motors for each side should be the ones that have the encoders plugged in
            Motors.can_talonsrx,
            Inversions.RIGHT,
            DriveLayout.DIFFERENTIAL
        );
    public static final DriveBase.ClosedLoopParams
        DRIVEBASE_PARAMS = new DriveBase.ClosedLoopParams(  // MAKE SURE THESE ARE ALL CORRECT BEFORE TEST DRIVING IT!!!
            0.0,        // use the 'empirical' value from characterization here rather than the actual width
            0.1524, // 6 inches, coverted to meters
            1.1185,
            2.1132,
            1.0668,
            3.5176,
            0.0,
            0.0,
            2.0,
            0.7,
            10.0,
            2.5,
            2.5,
            Inversions.BOTH
        );
	public static final int
		ARM_WINCH_CAN_ID = 10,
        ARM_EXTENDER_CAN_ID = 12,
		GRABBER_CAN_ID = 11,
		GRABBER_WRIST_PWM_PORT = 0
	;

    public static final double
        DRIVE_INPUT_DEADZONE = 0.05,
        DRIVE_INPUT_VEL_SCALE = -2.5,
        DRIVE_INPUT_EXP_POWER = 1.0,

        IMU_RATE_FILTER = 0.35,

        ACTIVE_PARK_VOLTS_PER_METER = 100.0,
        BALANCE_PARK_VOLTS_PER_DEGREE = 0.2,
        AUTO_PAD_ENGAGE_VELOCITY = 0.8,
        AUTO_PAD_INCLINE_VELOCITY = 0.1,

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
		GRABBER_W0_ANGLE_DEGREES = Manipulator.Grabber.grabWidthToAngle(0.0);
    ;
	
	public static final int		// see: https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
		SRX_MAG_UNITS_PER_REVOLUTION = 4096,
		FALCON_UNITS_PER_REVOLUTION = 2048,
        NEVEREST_UNITS_PER_REVOLUTION = (7 * 60 * 4),	// 7 pulse/rot * 60:1 gearing * 4 counts/pulse as counted by TalonSRX
        ANALOG_POT_UNITS_PER_REVOLUTION = 1024
	;


    // private static final HashMap<String, Command> AUTO_EVENTS = new HashMap<>();


}