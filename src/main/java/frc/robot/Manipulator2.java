package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Manipulator2 implements Subsystem, Sendable {

	public static final class Kinematics {

		/* ARM and WRIST */

		/* All measurements adhere to the common robot coordinate system standard as outlined here:
		 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html (+Z is up)
		 * The arm translations are in the same format but are transformed by whatever angle (pose) the arm is at. */

		public static final double	// METERS -- sourced from robot CAD model
			ARM_PIVOT_X = 0.143098,
			ARM_PIVOT_Z = 1.127109,
			ARM_V1_LINKAGE_LENGTH = 0.736541,
			ARM_V1_ELBOW_SIM_OFFSET = -0.004563,	// not applicable on actual robot, but needed to align the simulated model
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


		/* GRABBER */

		public static final double	// in INCHES because robot coordinate system conformity is not necessary
			GRAB_INITIAL_LINK_LENGTH = 5.085672,		// the distance between the main pivot (large gear) and the linkage connecting the finger
			GRAB_SURFACE_INSET_DIST = 1.509250,			// the inner translation from the finger pivot (connected to large gear) and the grab surface
			GRAB_PIVOT_CENTER_OFFSET = 3.777867 / 2.0,	// the distance between the centerline of the grabber plate and either large gear center
			GRAB_NEVEREST_GEARING = 28.0 / 12.0,		// the gearing between the neverest gear and the large gear(s)

			GRAB_A0_FINGER_WIDTH = getFingerWidth(0.0),		// the finger width at a standardized angle of 0.0
			GRAB_W0_PIVOT_ANGLE = getRequiredRotation(0.0);	// the pivot rotation when the fingers are touching

		/* These functions assume the grab rotation is standardized to a coordinate space with 0.0 being when the linkages are parallel
		 * and positive values representing an outward rotation (so with a max of ~125 and min of ~-5) */

		public static double getFingerWidth(double sdeg) {
			return (GRAB_PIVOT_CENTER_OFFSET - GRAB_SURFACE_INSET_DIST +
				(GRAB_INITIAL_LINK_LENGTH * Math.sin(Math.toRadians(sdeg)))) * 2.0;
		}
		public static double getRequiredRotation(double width) {
			double sine = ((width / 2.0) + GRAB_SURFACE_INSET_DIST - GRAB_PIVOT_CENTER_OFFSET) / GRAB_INITIAL_LINK_LENGTH;
			return sine >= 1.0 ? 90.0 : Math.toDegrees(Math.asin(sine));
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



	public static final class Arm implements Subsystem, Sendable {

		public static final LimitSwitchSource LIMIT_SWITCH_SOURCE = LimitSwitchSource.FeedbackConnector;
		public static final LimitSwitchNormal LIMIT_SWITCH_NORMALITY = LimitSwitchNormal.NormallyOpen;
		public static final FeedbackDevice WINCH_FEEDBACK_TYPE = FeedbackDevice.Analog;
		public static final int
			FB_UNITS_PER_RANGE = Constants.ANALOG_UNITS_PER_REVOLUTION,
			CL_IDX = 0;
		public static final double
			FB_RANGE_DEGREES = 270.0,
			TOP_ROTATION_ABSOLUTE = 821.0;	// << SET THIS TO THE ABSOLUTE MEASUREMENT WHEN TOP LIMIT IS TRIGGERED

		private final WPI_TalonSRX winch;

		public Arm(int id) {
			this.winch = new WPI_TalonSRX(id);
		}


		@Override
		public void periodic() {}
		@Override
		public void initSendable(SendableBuilder b) {
			
		}
	}

	public static abstract class Wrist implements Subsystem, Sendable {


		public static final class ServoImpl extends Wrist {

			public static final double
				MIN_PULSE_uS = 550.0,
				MAX_PULSE_uS = 2450.0,
				TOTAL_INPUT_RANGE = 270.0,
				GEARING = 3.0 / 2.0,
				TOTAL_OUTPUT_RANGE = TOTAL_INPUT_RANGE / GEARING,
				PARALLEL_OFFSET_ANGLE = 40.0,
				ABSOLUTE_MIN_ANGLE = -40.0,
				ABSOLUTE_MAX_ANGLE = 90.0,
				PARALLEL_OFFSET_PERCENT = PARALLEL_OFFSET_ANGLE / TOTAL_OUTPUT_RANGE,
				MIN_PERCENT = (ABSOLUTE_MIN_ANGLE + PARALLEL_OFFSET_ANGLE) / TOTAL_OUTPUT_RANGE,
				MAX_PERCENT = (ABSOLUTE_MAX_ANGLE + PARALLEL_OFFSET_ANGLE) / TOTAL_OUTPUT_RANGE;

			private final Servo servo;

		}
		public static final class SeatMotorImpl extends Wrist {
	
			private final WPI_TalonSRX main;

		}
	}

	public static abstract class Intake implements Subsystem, Sendable {

		protected final WPI_TalonSRX main;


		public static final class GrabberV1 extends Intake {

		}
		public static final class GrabberV2 extends Intake {

		}
		public static final class Wheeled extends Intake {

		}
	}



	@Override
	public void periodic() {}
	@Override
	public void initSendable(SendableBuilder b) {
		
	}

}
