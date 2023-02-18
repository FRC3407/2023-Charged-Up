package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.team3407.drive.*;
import frc.robot.team3407.drive.Types.*;


public final class Constants {

    public static final DriveMap_4<WPI_TalonSRX>
        DRIVEBASE_LAYOUT = new DriveMap_4<>(
            0, 1, 2, 3,			// note that the 'front' motors for each side should be the ones that have the encoders plugged in
            Motors.can_talonsrx,
            Inversions.NEITHER,
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
            Inversions.NEITHER
        );

    public static final double
        DRIVE_INPUT_DEADZONE = 0.08,
        DRIVE_INPUT_VEL_SCALE = -2.5,
        DRIVE_INPUT_EXP_POWER = 2.0,

        IMU_RATE_FILTER = 0.25,

        ACTIVE_PARK_VOLTS_PER_METER = 100.0,
        BALANCE_PARK_VOLTS_PER_DEGREE = 0.2,
        AUTO_PAD_INCLINE_VELOCITY = 0.2
    ;


	// define motor ports and sensor ids here


    // add voltage limits/params here


	// add physical properties here

	
	public static final double
		SRX_MAG_UNITS_PER_REVOLUTION = 4096,
		FALCON_UNITS_PER_REVOLUTION = 2048
	;


    private static final HashMap<String, Command> AUTO_EVENTS = new HashMap<>();


}