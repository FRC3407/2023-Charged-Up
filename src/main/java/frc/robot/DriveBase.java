package frc.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.team3407.drive.Types.*;


public class DriveBase implements Subsystem, Sendable {

    public static class ClosedLoopParams {
        public final Inversions
            encoder_inversions;
        public final double
            track_width_meters,
            wheel_diameter_meters
        ;
        public ClosedLoopParams(
            double trackwidth,
            double wheeldiameter,
            Inversions encoderinversions
        ) {
            this.track_width_meters = trackwidth;
            this.wheel_diameter_meters = wheeldiameter;
            this.encoder_inversions = encoderinversions;
        }
    }
    
    private Gyro 
        gyro;
    private WPI_TalonSRX
        left, left2,
        right, right2;
    private ClosedLoopParams
        parameters;

    public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params) {
        this.parameters = params;
        this.gyro = gy;
        this.left = map.front_left;
        this.right = map.front_right;
        this.left2 = map.back_left;
        this.right2 = map.back_right;

        this.left.configFactoryDefault();
        this.right.configFactoryDefault();
        this.left2.configFactoryDefault();
        this.right2.configFactoryDefault();
        this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
		this.left.setSensorPhase(parameters.encoder_inversions.left);
		this.right.setSensorPhase(parameters.encoder_inversions.right);
        this.left2.follow(this.left);
        this.right2.follow(this.right);
        this.left2.setInverted(InvertType.FollowMaster);
        this.right2.setInverted(InvertType.FollowMaster);
    }

    @Override
    public void periodic() {
        // update field position and other tethered stats
    }
    @Override
    public void initSendable(SendableBuilder b) {
        // add data to send
    }

    public void setDriveVoltage(double lv, double rv) {
        this.left.setVoltage(lv);
        this.right.setVoltage(rv);
    }
    public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
    public void zeroHeading() {
		this.gyro.reset();
	}


    public double getRawLeftPosition() {
		return this.left.getSelectedSensorPosition();
	}
	public double getRawRightPosition() {
		return this.right.getSelectedSensorPosition();
	}
	public double getRawLeftVelocity() {
		return this.left.getSelectedSensorVelocity();
	}
	public double getRawRightVelocity() {
		return this.right.getSelectedSensorVelocity();
	}

    /* !!!--> IN METERS AND METERS/SECOND <--!!! */
    public double getLeftPosition() {
		return this.getRawLeftPosition()				    // output is in encoder units...
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION		// to get total rotations
			* this.parameters.wheel_diameter_meters * Math.PI;	// to get total distance
	}
	public double getRightPosition() {
		return this.getRawRightPosition()				    // ^^^
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION
			* this.parameters.wheel_diameter_meters * Math.PI;
	}
	public double getLeftVelocity() {
		return this.getRawLeftVelocity()				    // output is in encoder units per 100 ms
			* 10											// to get encoder units per second
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION		// to get rotations per second
			* this.parameters.wheel_diameter_meters * Math.PI;	// to get meters per second
	}
	public double getRightVelocity() {
		return this.getRawRightVelocity()				    // ^^^
			* 10
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION
			* this.parameters.wheel_diameter_meters * Math.PI;
	}

    public double getContinuousAngle() {	// in degrees
		return this.gyro.getAngle();
	}
	public double getHeading() {			// from -180 to 180
		return this.gyro.getRotation2d().getDegrees();
	}
	public double getTurnRate() {	        // in degrees per second
		return -this.gyro.getRate();
	}
	public Rotation2d getRotation() {
		return this.gyro.getRotation2d();
	}


}