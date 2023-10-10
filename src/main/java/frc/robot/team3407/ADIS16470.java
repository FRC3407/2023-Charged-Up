package frc.robot.team3407;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.SPI;


// For some reason the ADIS16470_IMU driver does not implement Gyro, even though it already has the correct method names
public class ADIS16470 extends ADIS16470_IMU implements Gyro, Accelerometer {

	public ADIS16470() { super(); }
	public ADIS16470(IMUAxis yaw_axis, SPI.Port port, CalibrationTime cal_time) {
		super(yaw_axis, port, cal_time);
	}

	@Override public double getAngle() {
		return super.getAngle() * -1;	// convert from CCW positive to CW positive
	}
	@Override public double getRate() {
		return super.getRate() * -1;	// convert from CCW positive to CW positive
	}

	public double getX() {
		return super.getAccelX();
	}
	public double getY() {
		return super.getAccelY();
	}
	public double getZ() {
		return super.getAccelZ();
	}
	public void setRange(Range r) {

	}

	@Override
	public void initSendable(SendableBuilder b) {
		super.initSendable(b);
		b.addDoubleProperty("Rate", this::getRate, null);
		b.addDoubleProperty("X Accel", this::getX, null);
		b.addDoubleProperty("Y Accel", this::getY, null);
		b.addDoubleProperty("Z Accel", this::getZ, null);
		b.addDoubleProperty("X Comp Angle", super::getXComplementaryAngle, null);
		b.addDoubleProperty("Y Comp Angle", super::getYComplementaryAngle, null);
		b.addDoubleProperty("X Accel Angle", super::getXFilteredAccelAngle, null);
		b.addDoubleProperty("Y Accel Angle", super::getYFilteredAccelAngle, null);
	}


}