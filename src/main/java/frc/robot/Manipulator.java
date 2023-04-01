package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.*;


public final class Manipulator implements Sendable {

	public static final class Arm implements Subsystem, Sendable {

		/* TODO:
		 * Configure PIDF
		 */

		public static final LimitSwitchSource LIMIT_SWITCH_SOURCE = LimitSwitchSource.FeedbackConnector;
		public static final LimitSwitchNormal LIMIT_SWITCH_NORMALITY = LimitSwitchNormal.NormallyOpen;
		public static final FeedbackDevice WINCH_FEEDBACK_TYPE = FeedbackDevice.Analog;
		public static final int
			FB_UNITS_PER_ROTATION = Constants.ANALOG_POT_UNITS_PER_REVOLUTION,
			CONTROL_LOOP_IDX = 0;
		public static final boolean
			INVERT_ARM_ANGLE_ENCODER = false,
			CLEAR_ANGLE_ON_BOTTOM = false,
			CLEAR_ANGLE_ON_TOP = false;

		private final WPI_TalonSRX winch;
		// private final WPI_TalonSRX extender;

		public Arm(int id) {
			this.winch = new WPI_TalonSRX(id);

			this.winch.configFactoryDefault();
			this.winch.configSelectedFeedbackSensor(WINCH_FEEDBACK_TYPE, CONTROL_LOOP_IDX, 0);
			//this.winch.setSelectedSensorPosition(0.0, CONTROL_LOOP_IDX, 0);      // if the potentiometer is absolute, then we probably want the absolute value right?
			this.winch.setSensorPhase(INVERT_ARM_ANGLE_ENCODER);
			this.winch.setNeutralMode(NeutralMode.Brake);
			this.winch.configForwardLimitSwitchSource(LIMIT_SWITCH_SOURCE, LIMIT_SWITCH_NORMALITY);
			this.winch.configReverseLimitSwitchSource(LIMIT_SWITCH_SOURCE, LIMIT_SWITCH_NORMALITY);
			this.winch.configClearPositionOnLimitF(CLEAR_ANGLE_ON_TOP, 0);
			this.winch.configClearPositionOnLimitR(CLEAR_ANGLE_ON_BOTTOM, 0);
			this.winch.config_kF(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KF);
			this.winch.config_kP(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KP);
			this.winch.config_kI(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KI);
			this.winch.config_kD(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KD);
			this.winch.configMotionAcceleration(
				Constants.ARM_ANGLE_ACC_DEG_PER_SEC_SQRD / 360.0 * FB_UNITS_PER_ROTATION / 10.0);
			this.winch.configMotionCruiseVelocity(
				Constants.ARM_ANGLE_CRUISE_DEG_PER_SEC / 360.0 * FB_UNITS_PER_ROTATION / 10.0);
			// also see: config_IntegralZone(), configClosedLoopPeakOutput(), setStatusFramePeriod(), etc...
		}

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Arm Encoder Value Raw", this::getWinchRawPosition, null);
			b.addDoubleProperty("Arm Angle (degrees)", this::getWinchDegPosition, null);
			b.addDoubleProperty("Arm Angle Rate", this::getWinchDegVelocity, null);
			b.addBooleanProperty("Winch Lower Limit", ()->this.winch.isRevLimitSwitchClosed() == 1, null);
			b.addBooleanProperty("Winch Upper Limit", ()->this.winch.isFwdLimitSwitchClosed() == 1, null);
			b.addDoubleProperty("Winch Voltage", this.winch::getMotorOutputVoltage, null);
			b.addDoubleArrayProperty("Winch Current [In:Out]",
				()->{ return new double[]{
					this.winch.getSupplyCurrent(),
					this.winch.getStatorCurrent()
				}; }, null);
			b.addDoubleProperty("Winch MC Temp", this.winch::getTemperature, null);
		}

		public void setWinchVoltage(double v) {
			this.winch.setVoltage(v);
		}
		public void setWinchPosition(double deg) {
			this.winch.set(ControlMode.Position,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void setWinchPosition_MM(double deg) {
			this.winch.set(ControlMode.MotionMagic,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void resetPosition() {
			this.winch.setSelectedSensorPosition(0.0);
		}

		public double getWinchRawPosition() {
			return this.winch.getSelectedSensorPosition(CONTROL_LOOP_IDX);
		}
		public double getWinchRawVelocity() {
			return this.winch.getSelectedSensorVelocity(CONTROL_LOOP_IDX);
		}

		public double getWinchRotPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_ROTATION;
		}
		public double getWinchDegPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_ROTATION * 360.0;
		}
		public double getWinchRotVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION;
		}
		public double getWinchDegVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION * 360;
		}


	}
	public static final class Grabber implements Subsystem, Sendable {

		/* TODO:
		 * Determine grabber angle convention
		 * Determine homing location if any, soft limits, etc...
		 * Configure PIDF
		 */

		public static final double
			WRIST_MIN_PULSE_uS = 550.0,
			WRIST_MAX_PULSE_uS = 2450.0,
			WRIST_TOTAL_INPUT_RANGE = 270.0,	// range before gearing
			WRIST_GEARING = (3.0 / 2.0),
			WRIST_TOTAL_OUTPUT_RANGE = (WRIST_TOTAL_INPUT_RANGE / WRIST_GEARING),
			WRIST_PARALLEL_OFFSET = 40.0,	// should make setting wrist angle simpler if angles are based off of center (parallel to arm) angle
			WRIST_MIN_ANGLE = -40.0,
			WRIST_MAX_ANGLE = 90.0,
			WRIST_PARALLEL_PERCENT = WRIST_PARALLEL_OFFSET / WRIST_TOTAL_OUTPUT_RANGE,
			WRIST_MIN_PERCENT = (WRIST_MIN_ANGLE + WRIST_PARALLEL_OFFSET) / WRIST_TOTAL_OUTPUT_RANGE,
			WRIST_MAX_PERCENT = (WRIST_MAX_ANGLE + WRIST_PARALLEL_OFFSET) / WRIST_TOTAL_OUTPUT_RANGE,
			GRAB_MAX_ANGLE = 125.0;
		public static final int
			FB_UNITS_PER_ROTATION = (int)(Constants.NEVEREST_UNITS_PER_REVOLUTION * Constants.GRABBER_GEARING_IN2OUT),
			CONTROL_LOOP_IDX = 0;
		public static final boolean
			INVERT_WRIST_OUTPUT = true,
			INVERT_GRAB_ENCODER = false,
			ENABLE_GRAB_SOFT_FWD_LIMIT = true,
			CLEAR_GRAB_ANGLE_ON_RLIMIT = true;

		private final WPI_TalonSRX main;
		private final Servo wrist;

		public Grabber(int id, int schan) {
			this.main = new WPI_TalonSRX(id);
			this.wrist = new Servo(schan);

			this.wrist.setBounds(WRIST_MAX_PULSE_uS / 1000.0, 0, 0, 0, WRIST_MIN_PULSE_uS / 1000.0);
			this.main.configFactoryDefault();
			this.main.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, CONTROL_LOOP_IDX, 0);
			this.main.setSelectedSensorPosition(0.0, CONTROL_LOOP_IDX, 0);
			this.main.setSensorPhase(INVERT_GRAB_ENCODER);
			this.main.configForwardSoftLimitThreshold(GRAB_MAX_ANGLE / 360.0 * FB_UNITS_PER_ROTATION);		// 125 degrees max
			this.main.configReverseSoftLimitThreshold(0.0);								// dont let it go backwards
			this.main.configForwardSoftLimitEnable(ENABLE_GRAB_SOFT_FWD_LIMIT);
			this.main.configClearPositionOnLimitR(CLEAR_GRAB_ANGLE_ON_RLIMIT, 0);
			this.main.config_kF(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KF);
			this.main.config_kP(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KP);
			this.main.config_kI(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KI);
			this.main.config_kD(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KD);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Wrist Angle", this::getWristAngle, null);
			b.addDoubleProperty("Wrist Percent Output", this::getWristPercent, null);
			b.addDoubleProperty("Wrist PWM Raw", this.wrist::getRaw, null);
			b.addDoubleProperty("Grabber Rotation (degrees)", this::getGrabDegPosition, null);
			b.addDoubleProperty("Grabber Encoder Raw", this::getGrabRawPosition, null);
			b.addDoubleProperty("Grabber Rotation Rate", this::getGrabDegVelocity, null);
			b.addDoubleProperty("Grabber Width (inches)", this::getGrabWidth, null);
			b.addBooleanProperty("Grabber Reset Limit", ()->this.main.isRevLimitSwitchClosed() == 1, null);
			b.addDoubleProperty("Grab Motor Voltage", this.main::getMotorOutputVoltage, null);
			b.addDoubleArrayProperty("Grab Motor Current [In:Out]", ()->{
				return new double[]{
					this.main.getSupplyCurrent(),
					this.main.getStatorCurrent()
				}; }, null);
			b.addDoubleProperty("Grab MC Temp", this.main::getTemperature, null);
		}

		public static double grabAngleToWidth(double deg) {
			return 2.0 * (Constants.GRABBER_PIVOT_OFFSET_INCHES +
				Constants.GRABBER_ROT_RADIUS_INCHES * Math.sin(Math.toRadians(deg)) -
					Constants.GRABBER_FINGER_OFFSET_INCHES);
		}
		public static double grabWidthToAngle(double inches) {
			double ratio = (((inches / 2.0) + Constants.GRABBER_FINGER_OFFSET_INCHES -
				Constants.GRABBER_PIVOT_OFFSET_INCHES) / Constants.GRABBER_ROT_RADIUS_INCHES);
			if(ratio >= 1.0) { return 90.0; }
			return Math.toDegrees(Math.asin(ratio));
		}

		public void setGrabberVoltage(double v) {
			this.main.setVoltage(v);
		}
		public void setGrabberAngle(double deg) {	// also probably account for W0 being not perfectly at 0 degrees
			this.main.set(ControlMode.Position,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void setGrabberWidth(double inches) {	// also probably account for A0 being not perfectly at 0 inches
			this.main.set(ControlMode.Position,
				grabWidthToAngle(inches) / 360.0 * FB_UNITS_PER_ROTATION);
		}

		public void setWristPercent(double p) {
			p = Math.max(WRIST_MIN_PERCENT, Math.min(WRIST_MAX_PERCENT, p));
			if(INVERT_WRIST_OUTPUT) { p = 1.0 - p; }
			this.wrist.setPosition(p);
		}
		public void setWristAngle(double deg) {
			deg += WRIST_PARALLEL_OFFSET;
			if(deg < 0) { deg = 0; }
			if(deg > WRIST_TOTAL_OUTPUT_RANGE) { deg = WRIST_TOTAL_OUTPUT_RANGE; }
			this.setWristPercent(deg / WRIST_TOTAL_OUTPUT_RANGE);
		}
		public void setWristDisabled() {
			this.wrist.setDisabled();
		}

		public double getWristPercent() {
			return this.wrist.getPosition();
		}
		public double getWristAngle() {
			double v = this.wrist.getPosition();
			return (INVERT_WRIST_OUTPUT ? (1.0 - v) : v) * WRIST_TOTAL_OUTPUT_RANGE - WRIST_PARALLEL_OFFSET;
		}

		public double getGrabRawPosition() {
			return this.main.getSelectedSensorPosition(CONTROL_LOOP_IDX);
		}
		public double getGrabRawVelocity() {
			return this.main.getSelectedSensorVelocity(CONTROL_LOOP_IDX);
		}
		public double getGrabRotPosition() {
			return this.getGrabRawPosition() / FB_UNITS_PER_ROTATION;
		}
		public double getGrabDegPosition() {
			return this.getGrabRawPosition() / FB_UNITS_PER_ROTATION * 360.0;
		}
		public double getGrabWidth() {
			return grabAngleToWidth(this.getGrabDegPosition());
		}
		public double getGrabRotVelocity() {
			return this.getGrabRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION;
		}
		public double getGrabDegVelocity() {
			return this.getGrabRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION * 360.0;
		}

	}



	public final Arm arm;
	public final Grabber grabber;

	public Manipulator(Arm a, Grabber g) {
		this.arm = a;
		this.grabber = g;
	}

	@Override
	public void initSendable(SendableBuilder b) {

	}

	public void startLogging(String basekey) {
		SmartDashboard.putData(basekey, this);
		SmartDashboard.putData(basekey + "/Arm", this.arm);
		SmartDashboard.putData(basekey + "/Grabber", this.grabber);
	}


	public ManipulatorControl controlManipulator(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
	) {
		return new ManipulatorControl(this, a, g, w);
	}
	public ManipulatorControl controlManipulator(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
		BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl
	) {
		return new ManipulatorControl(this, a, g, w, wr, al, gl);
	}

	public ManipulatorControl2 controlManipulatorAdv(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
	) {
		return new ManipulatorControl2(this, a, g, w);
	}
	public ManipulatorControl2 controlManipulatorAdv(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
		BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl
	) {
		return new ManipulatorControl2(this, a, g, w, wr, al, gl);
	}





	public static class ManipulatorControl extends CommandBase {

		public static final double
			ARM_WINCH_VOLTAGE_SCALE = 5.0,
			ARM_WINCH_LOCK_VOLTAGE = 1.1,
			GRAB_CLAW_VOLTAGE_SCALE = 7.0,
			GRAB_CLAW_LOCK_VOLTAGE = 8.0,
			WRIST_NEUTRAL_SETPOINT = Grabber.WRIST_MAX_PERCENT,
			WRIST_ACCUMULATION_RATE_SCALE = 0.01;	// at full throttle, add 0.01 x 50 loops per second = 0.5 per second change [maximum]

		protected final Manipulator
			manipulator;
		protected final DoubleSupplier
			arm_rate,
			grab_rate,
			wrist_rate;
		protected final BooleanSupplier
			wrist_reset,
			arm_lock,
			grab_lock;

		protected double
			wrist_position = WRIST_NEUTRAL_SETPOINT;
		protected boolean
			is_arm_locked = false,
			is_grab_locked = false;


		public ManipulatorControl(
			Manipulator m,
			DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
		) {
			this.manipulator = m;
			this.arm_rate = a;
			this.grab_rate = g;
			this.wrist_rate = w;
			this.wrist_reset = this.arm_lock = this.grab_lock = ()->false;
			super.addRequirements(m.arm, m.grabber);
		}
		public ManipulatorControl(
			Manipulator m,
			DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
			BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl
		) {
			this.manipulator = m;
			this.arm_rate = a;
			this.grab_rate = g;
			this.wrist_rate = w;
			this.wrist_reset = wr;
			this.arm_lock = al;
			this.grab_lock = gl;
			super.addRequirements(m.arm, m.grabber);
		}


		@Override
		public void initialize() {
			this.wrist_position = WRIST_NEUTRAL_SETPOINT;
		}
		@Override
		public void execute() {
			this.wrist_position += this.wrist_rate.getAsDouble() * WRIST_ACCUMULATION_RATE_SCALE;
			if(this.wrist_reset.getAsBoolean()) {
				this.wrist_position = WRIST_NEUTRAL_SETPOINT;
			} else {
				this.wrist_position = Math.max(
					Grabber.WRIST_MIN_PERCENT, Math.min(
						Grabber.WRIST_MAX_PERCENT, this.wrist_position));
			}
			double
				arate = this.arm_rate.getAsDouble(),
				grate = this.grab_rate.getAsDouble();
			if(this.arm_lock.getAsBoolean()) { this.is_arm_locked = !this.is_arm_locked; }
			if(this.grab_lock.getAsBoolean()) { this.is_grab_locked = !this.is_grab_locked; }
			if(arate != 0) { this.is_arm_locked = false; }
			if(grate != 0) { this.is_grab_locked = false; }
			this.manipulator.arm.setWinchVoltage(this.is_arm_locked ? ARM_WINCH_LOCK_VOLTAGE : (arate * ARM_WINCH_VOLTAGE_SCALE));
			this.manipulator.grabber.setGrabberVoltage(this.is_grab_locked ? GRAB_CLAW_LOCK_VOLTAGE : (grate * GRAB_CLAW_VOLTAGE_SCALE));
			this.manipulator.grabber.setWristPercent(this.wrist_position);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		@Override
		public void end(boolean isfinished) {
			this.manipulator.arm.setWinchVoltage(0);
			this.manipulator.grabber.setGrabberVoltage(0);
			this.manipulator.grabber.setWristDisabled();
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Winch Voltage Setpoint",
				()->this.is_arm_locked ? ARM_WINCH_LOCK_VOLTAGE : (this.arm_rate.getAsDouble() * ARM_WINCH_VOLTAGE_SCALE), null);
			b.addDoubleProperty("Grabber Voltage Setpoint",
				()->this.is_grab_locked ? GRAB_CLAW_LOCK_VOLTAGE : (this.grab_rate.getAsDouble() * GRAB_CLAW_VOLTAGE_SCALE), null);
			b.addDoubleProperty("Wrist Position Setpoint", ()->this.wrist_position, null);
			b.addBooleanProperty("Arm Locked", ()->this.is_arm_locked, null);
			b.addBooleanProperty("Grab Locked", ()->this.is_grab_locked, null);
		}

	}



	public static class ManipulatorControl2 extends ManipulatorControl {

		public static final double
			ARM_ANGLE_OFFSET = -130.0,
			ARM_PARK_UPPER_ANGLE_REL = 15.0;
		public static final boolean
			ENABLE_WRIST_PARK_BOUND_LIMITING = true;

		private double
			wrist_pos_offset = 0.0,
			arm_rel_angle = 0.0,
			wrist_hz_angle = 0.0,
			wrist_bounded_lower_limit = 0.0;

		public ManipulatorControl2(
			Manipulator m,
			DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
		) {
			super(m, a, g, w);
		}
		public ManipulatorControl2(
			Manipulator m,
			DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
			BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl
		) {
			super(m, a, g, w, wr, al, gl);
		}

		public double getRelArmAngle() {
			return super.manipulator.arm.getWinchDegPosition() + ARM_ANGLE_OFFSET;
		}
		public double getRelWristAngle() {
			return super.manipulator.grabber.getWristAngle();
		}

		@Override
		public void initialize() {
			this.wrist_pos_offset = 0.0;
			super.wrist_position = 0.0;
		}
		@Override
		public void execute() {
			this.arm_rel_angle = this.getRelArmAngle();
			this.wrist_hz_angle = 90.0 - this.arm_rel_angle;	// if the arm is 0deg relative, then the wrist should be 90, if the arm is 90deg relative, the wrist should be 0
			this.wrist_pos_offset += super.wrist_rate.getAsDouble() * WRIST_ACCUMULATION_RATE_SCALE * Grabber.WRIST_TOTAL_OUTPUT_RANGE;
			if(super.wrist_reset.getAsBoolean()) {
				this.wrist_pos_offset = 0.0;
			} else {
				this.wrist_bounded_lower_limit = Grabber.WRIST_MIN_ANGLE;
				if(ENABLE_WRIST_PARK_BOUND_LIMITING && this.arm_rel_angle < ARM_PARK_UPPER_ANGLE_REL) {
					this.wrist_bounded_lower_limit = (ARM_PARK_UPPER_ANGLE_REL - this.arm_rel_angle) * 8.0 - 30.0;	// [inverted based on threshold] arm angle from vertical  / threshold range * 120 degrees wrist range - 30.0 degrees lowest wrist angle
				}
				this.wrist_pos_offset = Math.max(
					this.wrist_bounded_lower_limit - this.wrist_hz_angle, Math.min(
						Grabber.WRIST_MAX_ANGLE - this.wrist_hz_angle, this.wrist_pos_offset));	// keep the offset inside the normal bounds, shifted by the hz angle
			}
			super.wrist_position = this.wrist_pos_offset + this.wrist_hz_angle;

			double arate = super.arm_rate.getAsDouble();
			double grate = super.grab_rate.getAsDouble();
			if(super.arm_lock.getAsBoolean()) { super.is_arm_locked = !super.is_arm_locked; }
			if(super.grab_lock.getAsBoolean()) { super.is_grab_locked = !super.is_grab_locked; }
			if(arate != 0.0) { super.is_arm_locked = false; }
			if(grate != 0.0) { super.is_grab_locked = false; }
			super.manipulator.arm.setWinchVoltage(super.is_arm_locked ? ARM_WINCH_LOCK_VOLTAGE : (arate * ARM_WINCH_VOLTAGE_SCALE));
			super.manipulator.grabber.setGrabberVoltage(super.is_grab_locked ? GRAB_CLAW_LOCK_VOLTAGE : (grate * GRAB_CLAW_VOLTAGE_SCALE));
			super.manipulator.grabber.setWristAngle(super.wrist_position);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Position Offset", ()->this.wrist_pos_offset, null);
			b.addDoubleProperty("Arm Relative Angle", ()->this.arm_rel_angle, null);
			b.addDoubleProperty("Wrist Horizontal Angle", ()->this.wrist_hz_angle, null);
			b.addDoubleProperty("Bumper Bounded Wrist Limit", ()->this.wrist_bounded_lower_limit, null);
		}

	}

}
