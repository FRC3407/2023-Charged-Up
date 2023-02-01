package frc.robot;

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
	public static final Inversions
		DRIVEBASE_ENCODER_INVERSIONS = Inversions.NEITHER;


	// define motor ports and sensor ids here


    // add voltage limits/params here


	// add physical properties here

	
	public static final double
		SRX_MAG_UNITS_PER_REVOLUTION = 4096,
		FALCON_UNITS_PER_REVOLUTION = 2048
	;


}