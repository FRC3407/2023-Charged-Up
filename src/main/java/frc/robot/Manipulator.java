package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.*;


public final class Manipulator implements Subsystem, Sendable {

	/* PHYSICAL CONSTANTS:
		 * 	"PIVOT_X_METERS" -- how far forward from the robot's center is the pivot rod
		 * 	"PIVOT_Z_METERS" -- how high from the ground is the pivot rod
		 * ARM:
		 * 	"TOP_ROTATION_ABSOLUTE" -- the absolute fb degree amount when the arm is triggering the top limit
		 * 	"ARM_V1_LINKAGE_LENGTH" -- the arm (v1) distance between pivot points
		 * 	"ARM_V2_LINKAGE_LENGTH" -- the arm (v2) distance between pivot points
		 * 	"ARM_V2_ELBOW_HEIGHT" -- the arm (v2) height offset of the end pivot
		 * 	"ARM_ANGLE_OFFSET_TO_TOP" -- the degree range between the top triggering angle and when hanging verticle, used to transform relative coord spaces
		 * GRABBER:
		 * 	
		 */

	public static final class Arm implements Subsystem, Sendable {

		/* TODO:
		 * Configure PIDF
		 */

		public static final LimitSwitchSource LIMIT_SWITCH_SOURCE = LimitSwitchSource.FeedbackConnector;
		public static final LimitSwitchNormal LIMIT_SWITCH_NORMALITY = LimitSwitchNormal.NormallyOpen;
		public static final FeedbackDevice WINCH_FEEDBACK_TYPE = FeedbackDevice.Analog;
		public static final int
			FB_UNITS_PER_RANGE = Constants.ANALOG_UNITS_PER_REVOLUTION,
			CONTROL_LOOP_IDX = 0;
		public static final boolean
			INVERT_ARM_ENCODER = false,
			CLEAR_ANGLE_ON_BOTTOM = true,
			CLEAR_ANGLE_ON_TOP = true,
			DISABLE_CONTINUOUS_ANGLE = true;
		public static final double
			FB_RANGE_DEGREES = 270.0,
			TOP_ROTATION_ABSOLUTE = 625.0,	// << SET THIS TO THE ABSOLUTE MEASUREMENT WHEN TOP LIMIT IS TRIGGERED
			TOP_ROTATION_DEGREES = TOP_ROTATION_ABSOLUTE / FB_UNITS_PER_RANGE * FB_RANGE_DEGREES;

		private final WPI_TalonSRX winch;
		// private final WPI_TalonSRX extender;


		public Arm(int id) {
			this.winch = new WPI_TalonSRX(id);

			this.winch.configFactoryDefault();
			this.winch.configSelectedFeedbackSensor(WINCH_FEEDBACK_TYPE, CONTROL_LOOP_IDX, 0);
			this.winch.configFeedbackNotContinuous(DISABLE_CONTINUOUS_ANGLE, 0);
			this.winch.setSensorPhase(INVERT_ARM_ENCODER);
			// this.winch.setSelectedSensorPosition(this.winch.getSelectedSensorPosition() - TOP_ROTATION_ABSOLUTE, CONTROL_LOOP_IDX, 0);
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
				Constants.ARM_ANGLE_ACC_DEG_PER_SEC_SQRD / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE / 10.0);
			this.winch.configMotionCruiseVelocity(
				Constants.ARM_ANGLE_CRUISE_DEG_PER_SEC / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE / 10.0);
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
				deg / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE);
		}
		public void setWinchPosition_MM(double deg) {
			this.winch.set(ControlMode.MotionMagic,
				deg / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE);
		}
		public void setPosition(double sval) {
			this.winch.setSelectedSensorPosition(sval);
		}

		public double getWinchRawPosition() {
			return this.winch.getSelectedSensorPosition(CONTROL_LOOP_IDX);
		}
		public double getWinchRawVelocity() {
			return this.winch.getSelectedSensorVelocity(CONTROL_LOOP_IDX);
		}

		public double getWinchRotPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_RANGE;
		}
		public double getWinchDegPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_RANGE * FB_RANGE_DEGREES;
		}
		public double getWinchRotVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_RANGE;
		}
		public double getWinchDegVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_RANGE * FB_RANGE_DEGREES;
		}


	}
	public static final class Grabber implements Subsystem, Sendable {

		public static final class SeatMotorCounter {

			private final Counter counter;

			public SeatMotorCounter(int dio) {
				this.counter = new Counter(dio);
			}

			public int update(boolean fwd, boolean reset) {
				this.counter.setReverseDirection(!fwd);	// might not work
				if(reset) {
					this.counter.reset();
				}
				return this.counter.get();
			}
			public int get() {
				return this.counter.get();
			}

		}

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
			WRIST_PARALLEL_OFFSET = 65.0,	// translates the absolute rotation so that 0 deg is when the hand is parallel with the arm
			WRIST_MIN_ANGLE = -40.0,		// in arm-standardized coord space
			WRIST_MAX_ANGLE = 90.0,			// in arm-standardized coord space
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
			ENABLE_GRAB_SOFT_FWD_LIMIT = false,
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



	public static final class Kinematics {

		/* All measurements adhere to the common robot coordinate system standard as outlined here:
		 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html (+Z is up)
		 * The arm translations are in the same format but are transformed by whatever angle (pose) the arm is at. */

		public static final double	// meters -- sourced from robot CAD model
			ARM_PIVOT_X = 0.143098,
			ARM_PIVOT_Z = 1.127109,
			ARM_V1_LINKAGE_LENGTH = 0.736541,
			ARM_V1_ELBOW_SIM_OFFSET = -0.004563,
			ARM_V2_LINKAGE_LENGTH = 0.800100,
			ARM_V2_ELBOW_HEIGHT = 0.044450;

		public static final Translation3d
			ROBOT_ORIGIN_TO_PIVOT = new Translation3d(ARM_PIVOT_X, 0, ARM_PIVOT_Z),
			ARM_V1_LINKAGE_TRANSLATION = new Translation3d(ARM_V1_ELBOW_SIM_OFFSET, 0, ARM_V1_LINKAGE_LENGTH),
			ARM_V2_LINKAGE_TRANSLATION = new Translation3d(ARM_V2_ELBOW_HEIGHT, 0, ARM_V2_LINKAGE_LENGTH);


		/* These functions assume a standard coordinate base of 0.0 degrees for the arm translating to when it is hanging vertical,
		 * and a base of 0.0 degrees for the elbow when it is parallel with the arm. */

		public static Rotation3d getArmRotation3d(double arm_angle) {
			return new Rotation3d(0, Math.toRadians(180.0 - arm_angle), 0.0);
		}
		public static Pose3d getArmPose3d(double arm_angle) {
			return new Pose3d(ROBOT_ORIGIN_TO_PIVOT, getArmRotation3d(arm_angle));
		}
		public static Rotation3d getHandRotation3d(double arm_angle, double elbow_angle) {
			return getArmRotation3d(arm_angle).plus(new Rotation3d(0, Math.toRadians(90.0 - elbow_angle), 0.0));
		}
		public static Pose3d getHandV1Pose3d(double arm_angle, double elbow_angle) {
			return new Pose3d(
				ROBOT_ORIGIN_TO_PIVOT.plus(ARM_V1_LINKAGE_TRANSLATION.rotateBy(getArmRotation3d(arm_angle))),
				getHandRotation3d(arm_angle, elbow_angle));
		}
		public static Pose3d getHandV2Pose3d(double arm_angle, double elbow_angle) {
			return new Pose3d(
				ROBOT_ORIGIN_TO_PIVOT.plus(ARM_V2_LINKAGE_TRANSLATION.rotateBy(getArmRotation3d(arm_angle))),
				getHandRotation3d(arm_angle, elbow_angle));
		}

	}



	public static class ManipulatorPose implements Interpolatable<ManipulatorPose> {

		public double	// degrees
			arm_angle,		// relative from when the arm is hanging vertical - positive values represent outward movement
			elbow_angle;	// relative from being parallel with the arm -- positive values represent upward movement

		ManipulatorPose(double aa, double ea) {
			this.arm_angle = aa;
			this.elbow_angle = ea;
		}

		public void setElbowAbsolute(double ea) {
			this.elbow_angle = 90.0 - this.arm_angle - ea;
		}

		public Rotation3d armRotation3d() { return Kinematics.getArmRotation3d(this.arm_angle); }
		public Pose3d armPose3d() { return Kinematics.getArmPose3d(this.arm_angle); }
		public Rotation3d handRotation3d() { return Kinematics.getHandRotation3d(this.arm_angle, this.elbow_angle); }
		public Pose3d handV1Pose3d() { return Kinematics.getHandV1Pose3d(this.arm_angle, this.elbow_angle); }
		public Pose3d handV2Pose3d() { return Kinematics.getHandV2Pose3d(this.arm_angle, this.elbow_angle); }

		private double[] getRawPoseData(Pose3d hand) {
			Pose3d arm = this.armPose3d();
			Quaternion aq = arm.getRotation().getQuaternion();
			Quaternion hq = hand.getRotation().getQuaternion();
			return new double[]{
				arm.getX(), arm.getY(), arm.getZ(), aq.getW(), aq.getX(), aq.getY(), aq.getZ(),
				hand.getX(), hand.getY(), hand.getZ(), hq.getW(), hq.getX(), hq.getY(), hq.getZ()
			};
		}
		public double[] getV1RawPoseData() {
			return this.getRawPoseData(this.handV1Pose3d());
		}
		public double[] getV2RawPoseData() {
			return this.getRawPoseData(this.handV2Pose3d());
		}

		@Override
		public ManipulatorPose interpolate(ManipulatorPose end, double t) {
			return new ManipulatorPose(
				MathUtil.interpolate(this.arm_angle, end.arm_angle, t),
				MathUtil.interpolate(this.elbow_angle, end.elbow_angle, t)
			);
		}

	}
	public static class ManipulatorState implements Interpolatable<ManipulatorState> {

		public final ManipulatorPose aquisition_pose;
		public double aquisition_voltage;

		public ManipulatorState(double aa, double ea, double v) {
			this(new ManipulatorPose(aa, ea), v);
		}
		public ManipulatorState(ManipulatorPose p, double v) {
			this.aquisition_pose = p;
			this.aquisition_voltage = v;
		}

		@Override
		public ManipulatorState interpolate(ManipulatorState end, double t) {
			return new ManipulatorState(
				this.aquisition_pose.interpolate(end.aquisition_pose, t),
				MathUtil.interpolate(this.aquisition_voltage, end.aquisition_voltage, t)
			);
		}

	}


	public static final double
		ARM_ANGLE_OFFSET_TO_TOP = 102.0;

	public final Arm arm;
	public final Grabber grabber;

	public Manipulator(Arm a, Grabber g) {
		this.arm = a;
		this.grabber = g;
	}

	@Override
	public void periodic() {

	}
	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleProperty("Standardized Arm Angle", this::getStandardizedArmAngle, null);
		b.addDoubleArrayProperty("Component Poses 3D", this::getRawComponentData, null);
	}

	public void startLogging(String basekey) {
		SmartDashboard.putData(basekey, this);
		SmartDashboard.putData(basekey + "/Arm", this.arm);
		SmartDashboard.putData(basekey + "/Grabber", this.grabber);
	}



	public double getStandardizedArmAngle() {
		return this.arm.getWinchDegPosition() - Arm.TOP_ROTATION_DEGREES + ARM_ANGLE_OFFSET_TO_TOP;
	}
	public double getStandardizedHandAngle() {
		return this.grabber.getWristAngle();	// angle is already standardized to 0.0 when parallel with arm, see Grabber.WRIST_PARALLEL_OFFSET
	}
	public ManipulatorPose getDetectedPose() {
		return new ManipulatorPose(
			this.getStandardizedArmAngle(),
			this.getStandardizedHandAngle()
		);
	}
	private double[] getRawComponentData() {
		return this.getDetectedPose().getV1RawPoseData();
	}

	public void sendSetPoint(ManipulatorPose p) {
		this.arm.setWinchPosition_MM(p.arm_angle);	// apply transform if necessary
		this.grabber.setWristAngle(p.elbow_angle);	// apply transform if necessary
	}
	public void sendSetPoint(ManipulatorState s) {
		this.sendSetPoint(s.aquisition_pose);
		this.grabber.setGrabberVoltage(s.aquisition_voltage);
	}



	public ManipulatorControl controlManipulator(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
	) {
		return new ManipulatorControl(this, a, g, w);
	}
	public ManipulatorControl controlManipulator(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
		BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl,
		BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
	) {
		return new ManipulatorControl(this, a, g, w, wr, al, gl, wheelIntakeRight, wheelIntakeLeft);
	}

	public ManipulatorControl2 controlManipulatorAdv(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
	) {
		return new ManipulatorControl2(this, a, g, w);
	}
	public ManipulatorControl2 controlManipulatorAdv(
		DoubleSupplier a, DoubleSupplier g, DoubleSupplier w,
		BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl,
		BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
	) {
		return new ManipulatorControl2(this, a, g, w, wr, al, gl, wheelIntakeRight, wheelIntakeLeft);
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
			BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl,
			BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
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
			// ARM_ANGLE_OFFSET = -130.0,
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
			BooleanSupplier wr, BooleanSupplier al, BooleanSupplier gl,
			BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
		) {
			super(m, a, g, w, wr, al, gl, wheelIntakeRight, wheelIntakeLeft);
		}

		public double getRelArmAngle() {
			return super.manipulator.getStandardizedArmAngle();
		}
		public double getRelWristAngle() {
			return super.manipulator.getStandardizedHandAngle();
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
