package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.team3407.SenderNT;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.imageio.plugins.tiff.TIFFDirectory;
import javax.lang.model.util.ElementScanner14;

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
			ARM_V1_ELBOW_SIM_OFFSET = 0.004563,	// not applicable on actual robot, but needed to align the simulated model
			ARM_V2_LINKAGE_LENGTH = 0.800100,
			ARM_V2_ELBOW_HEIGHT = -0.044450;

		public static final Translation3d
			ROBOT_ORIGIN_TO_PIVOT = new Translation3d(ARM_PIVOT_X, 0, ARM_PIVOT_Z),
			ARM_V1_LINKAGE_TRANSLATION = new Translation3d(-ARM_V1_ELBOW_SIM_OFFSET, 0, ARM_V1_LINKAGE_LENGTH),
			ARM_V2_LINKAGE_TRANSLATION = new Translation3d(-ARM_V2_ELBOW_HEIGHT, 0, ARM_V2_LINKAGE_LENGTH);


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

		private static double[] getComponentData(double aa, Pose3d hand) {
			Pose3d arm = getArmPose3d(aa);
			Quaternion aq = arm.getRotation().getQuaternion(), hq = hand.getRotation().getQuaternion();
			return new double[]{
				arm.getX(), arm.getY(), arm.getZ(), aq.getW(), aq.getX(), aq.getY(), aq.getZ(),
				hand.getX(), hand.getY(), hand.getZ(), hq.getW(), hq.getX(), hq.getY(), hq.getZ()
			};
		}
		public static double[] getV1ComponentData(double aa, double ea) {
			return getComponentData(aa, getHandV1Pose3d(aa, ea));
		}
		public static double[] getV2ComponentData(double aa, double ea) {
			return getComponentData(aa, getHandV2Pose3d(aa, ea));
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

		public static final class HandBBox2d {

			/* This model is used for calculating a safe minimum hand angle (in arm coord space) such that the
			 * bumpers and ground are always cleared. Within the computations, +x points forward and +y points
			 * upward such that the robot is being viewed from the right side, and rotations follow the unit
			 * circle (CCW). The origin is the robot's center (same as in 3d), and y=0 also represents the ground plane. */

			public static final double	// in meters
				HAND_LOWER_BOUND = 0.05,
				HAND_LENGTH_BOUND = 0.5,
				BUMPER_Z_HEIGHT = 0.191262,
				BUMPER_X_LENGTH = 0.511937;
			
			public static final Translation2d
				ROBOT_ORIGIN_TO_PIVOT_2D = new Translation2d(ARM_PIVOT_X, ARM_PIVOT_Z),
				ARM_V1_LINKAGE_TRANSLATION_2D = new Translation2d(ARM_V1_ELBOW_SIM_OFFSET, -ARM_V1_LINKAGE_LENGTH),
				ARM_V2_LINKAGE_TRANSLATION_2D = new Translation2d(ARM_V2_ELBOW_HEIGHT, -ARM_V2_LINKAGE_LENGTH),
				BUMPER_CORNER_TRANSLATION_2D = new Translation2d(BUMPER_X_LENGTH, BUMPER_Z_HEIGHT);

			private static double calcAngle(double arm_angle, Translation2d arm_translation) {

				Rotation2d arm_rotation = Rotation2d.fromDegrees(arm_angle);
				Translation2d hand_pivot = ROBOT_ORIGIN_TO_PIVOT_2D.plus(arm_translation.rotateBy(arm_rotation));
				Translation2d to_bumper = BUMPER_CORNER_TRANSLATION_2D.minus(hand_pivot);

				double distance_to_bumpers = to_bumper.getNorm();	// direct distance from hand pivot to bumper corner
				double lateral_distance = Math.sqrt(distance_to_bumpers * distance_to_bumpers - HAND_LOWER_BOUND * HAND_LOWER_BOUND);	// hand lateral distance along the direct distance (the direct distance is the hypotenuse of the hand bbox)
				
				if(lateral_distance >= HAND_LENGTH_BOUND &&		// if the lateral distance is greater than the hand's lateral bound, and the pivot is beyond the bumpers...
					hand_pivot.getX() >= BUMPER_X_LENGTH) { return -180.0; }		// we can assume the hand will not hit anything.

				double from_horizontal = to_bumper.getAngle().getRadians() + Math.acos(lateral_distance / distance_to_bumpers);		// calculate the hand's angle compared to the x-axis
				if(HAND_LENGTH_BOUND * -Math.sin(from_horizontal) + HAND_LOWER_BOUND * -Math.cos(from_horizontal) > hand_pivot.getY()) {	// if the pivot is low enough that the hand could possibly hit the ground...
					double to_ground = -Math.asin(hand_pivot.getY() / Math.hypot(HAND_LOWER_BOUND, HAND_LENGTH_BOUND)) + Math.atan2(HAND_LOWER_BOUND, HAND_LENGTH_BOUND);	// calculate the hand's angle to clear the ground
					if(to_ground > from_horizontal) {	// we are finding a minimum angle, so use the higher value as it will be the most inclusive
						from_horizontal = to_ground;
					}
				}
				return 90.0 + Math.toDegrees(from_horizontal) - arm_angle;	// Add 90 degrees to convert to arm coord space, subtract the arm angle to make it relative.

			}

			public static double handV1MinAngle(double arm_angle) { return calcAngle(arm_angle, ARM_V1_LINKAGE_TRANSLATION_2D); }
			public static double handV2MinAngle(double arm_angle) { return calcAngle(arm_angle, ARM_V2_LINKAGE_TRANSLATION_2D); }

		}
		
	}
	public static final class Dynamics {

		public static final double WINCH_GEARING = 20.0;
		public static final DCMotor WINCH_MOTOR = DCMotor.getCIM(1).withReduction(WINCH_GEARING);
		public static final Translation2d
			ARM_LINKAGE_TRANSLATION = Kinematics.HandBBox2d.ARM_V1_LINKAGE_TRANSLATION_2D,
			ARM_CENTER_OF_MASS = new Translation2d(0.002827, -0.3750491),
			HAND_CENTER_OF_MASS = new Translation2d(0.152717, -0.024010),	// relative to the wrist
			ARM_HOOK_TRANSLATION = new Translation2d(-0.046038, 0.142240),	// x=stick-out, y=length along arm
			ROPE_FEED_TRANSLATION = new Translation2d(-0.141321, -0.085822),	// the x,y where the rope is routed to the hook, relative to the pivot
			SPRING_BOLT_TRANSLATION = new Translation2d(-0.022225, -0.139700),
			SPRING_CENTER_TRANSLATION = new Translation2d(-0.054873, -0.060991);
		public static final double
			ARM_MASS = 2.099,			// kg
			ARM_PIVOT_RI = 0.479,		// kgm^2
			HAND_MASS = 1.806,			// kg
			HAND_CG_RI = 0.039,			// kgm^2
			WINCH_FREE_RI = 0.01,		// my best attempt at an educated guess - I have no idea if this is anywhere close to realistic, although this value doesn't really matter since the system will self-stabilize
			ARM_HOOK_RADIUS = ARM_HOOK_TRANSLATION.getNorm(),
			WINCH_SPOOL_RADIUS = 0.0127,
			SPRING_PULL_RADIUS = SPRING_BOLT_TRANSLATION.getNorm(),
			SPRING_FORCE = 73.4,		// newtons (16.5 lbs)
			ARM_FRICTIONAL_TORQUE = 0.1,
			// WINCH_STATIC_TORQUE = 10.0,
			MIN_REAL_ARM_ANGLE = -8.0,
			MAX_REAL_ARM_ANGLE = 102.0;		// "Real" because sometimes the sensor readings go beyond these limits. These are the physically measured bounds.


		public static Translation2d getArmCG(double wrist_angle) {	// the location returned will need to be rotated by the arm angle for the absolute (x, y) COM
			Translation2d hand_cg = ARM_LINKAGE_TRANSLATION.plus(		// find the hand's COM in the arm's coordinate space
				HAND_CENTER_OF_MASS.rotateBy(Rotation2d.fromDegrees(wrist_angle - 90.0)));
			return hand_cg.times(HAND_MASS).plus(ARM_CENTER_OF_MASS.times(ARM_MASS)).div(HAND_MASS + ARM_MASS);	// weighted average of the COMs using the component masses
		}
		public static Translation2d getArmCG(double arm_angle, double wrist_angle) {	// this returns the CG position x,y (x,z in 3d) considering the arm's angle
			return getArmCG(wrist_angle).rotateBy(Rotation2d.fromDegrees(arm_angle));
		}
		public static double getArmRI(double wrist_angle) {
			return ARM_PIVOT_RI + HAND_CG_RI + HAND_MASS * Math.pow(
				HAND_CENTER_OF_MASS.rotateBy(Rotation2d.fromDegrees(wrist_angle - 90.0)).plus(ARM_LINKAGE_TRANSLATION).getNorm(), 2);
					// ^ get the hand's center of mass (x, y) compared to the pivot point, then then normal will be the radius
		}

		public static double estimateSpringTorque(double arm_angle) {
			return SPRING_FORCE * SPRING_PULL_RADIUS * Math.cos(Math.toRadians((arm_angle + 5.0) / 110.0 * 90.0));
		}
		public static double getSpringTorque(double arm_angle) {
			Translation2d bolt = SPRING_BOLT_TRANSLATION.rotateBy(Rotation2d.fromDegrees(arm_angle));
			Translation2d delta = bolt.minus(SPRING_CENTER_TRANSLATION);
			double cos = (bolt.getX() * delta.getX() + bolt.getY() * delta.getY()) / (bolt.getNorm() * delta.getNorm());
			return SPRING_FORCE * SPRING_PULL_RADIUS * Math.sqrt(1 - cos * cos);
		}
		public static double getGravitationalTorque(Translation2d cg) {
			return 9.8 * (ARM_MASS + HAND_MASS) * cg.getX();	// multiply by radius, then multiply by cos(theta), which equals x/radius, so the radius cancels
		}
		public static double getGravitationalTorque(double arm_angle, double wrist_angle) {
			return getGravitationalTorque(getArmCG(arm_angle, wrist_angle));
		}
		public static double getForwardAccTorque(double acc, Translation2d cg) {
			return -acc * (ARM_MASS + HAND_MASS) * cg.getY();
		}
		public static double getForwardAccTorque(double acc, double arm_angle, double wrist_angle) {
			return getForwardAccTorque(acc, getArmCG(arm_angle, wrist_angle));
		}

		public static double getEffectivePullRadius(double arm_angle) {
			Translation2d hook = ARM_HOOK_TRANSLATION.rotateBy(Rotation2d.fromDegrees(arm_angle));
			Translation2d delta = hook.minus(ROPE_FEED_TRANSLATION);
			double cos = (hook.getX() * delta.getX() + hook.getY() * delta.getY()) / (hook.getNorm() * delta.getNorm());
			return ARM_HOOK_RADIUS * Math.sqrt(1 - cos * cos);
		}

		public static double ropeDelta(double arm_angle) {
			return ARM_HOOK_TRANSLATION.rotateBy(Rotation2d.fromDegrees(arm_angle)).minus(ROPE_FEED_TRANSLATION).getNorm();
		}
		/** See here for derivation: https://www.derivative-calculator.net/#expr=%5Csqrt%7B%5Cleft%28c%5Ccos%5Cleft%28%5Ctheta%5Cright%29-d%5Csin%5Cleft%28%5Ctheta%5Cright%29-a%5Cright%29%5E%7B2%7D%2B%5Cleft%28c%5Csin%5Cleft%28%5Ctheta%5Cright%29%2Bd%5Ccos%5Cleft%28%5Ctheta%5Cright%29-b%5Cright%29%5E%7B2%7D%7D&diffvar=theta&difforder=2&simplify=1 */
		public static double ropeDeltaPrimePrime(double arm_angle) {
			final double
				sin = Math.sin(Math.toRadians(arm_angle)),
				cos = Math.cos(Math.toRadians(arm_angle)),
				a = ROPE_FEED_TRANSLATION.getX(),
				b = ROPE_FEED_TRANSLATION.getY(),
				c = ARM_HOOK_TRANSLATION.getX(),
				d = ARM_HOOK_TRANSLATION.getY(),

				sin2 = (sin * sin),
				cos2 = (cos * cos),
				c2 = (c * c),
				d2 = (d * d),

				m = (a*d*sin - b*c*sin - a*(c*cos - a) - b*(d*cos - b)),
				n = (d2*sin2 + c2*sin2 + a*d*sin - b*c*sin + d2*cos2 + c2*cos2 - b*d*cos - a*c*cos),
				j = (d*sin - c*cos + a),
				k = (c*sin + d*cos - b),
				o = (j*j + k*k);

			return -(m * n) / Math.pow(o, 1.5);
		}


		public static final class Integrator {

			private Matrix<N1, N1> u_input;		// volts
			private Matrix<N2, N1> x_state;
			private Matrix<N2, N1> y_output;

			// private Matrix<N2, N2> A_matrix;
			private Matrix<N2, N1> B_matrix;
			// The output is equal to the state, so we don't need a C or D matrix.

			private double wrist_angle = 0.0;
			private double sys_MOI = getArmRI(0.0);


			public Integrator(double init_angle) {
				this.u_input = Matrix.mat(Nat.N1(), Nat.N1()).fill(0.0);
				this.x_state = Matrix.mat(Nat.N2(), Nat.N1()).fill(init_angle, 0.0);	// initial theta of the provided angle, and omega of 0
				this.y_output = this.x_state;
				// this.A_matrix = new Matrix<>(Nat.N2(), Nat.N2());
				this.B_matrix = Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0);
			}

			public void setInputs(double volts, double wrist_angle) {
				this.u_input.set(0, 0, volts);	// limit to battery voltage?
				this.wrist_angle = wrist_angle;
				this.sys_MOI = getArmRI(this.wrist_angle);
			}
			public void update(double dt_seconds) {
				this.x_state = NumericalIntegration.rk4(this::dynamics, this.x_state, this.u_input, dt_seconds);
				this.y_output = this.x_state;
			}

			public double getAngle() {
				return Math.toDegrees(this.y_output.get(0, 0));
			}
			public double getVelocity() {
				return Math.toDegrees(this.y_output.get(1, 0));
			}
			// method for simulated current draw


			protected Matrix<N2, N1> dynamics(Matrix<N2, N1> x, Matrix<N1, N1> u) {

				final double omega_grav = getGravitationalTorque(x.get(0, 0), this.wrist_angle) / sys_MOI;
				final double omega_spring = estimateSpringTorque(x.get(0, 0)) / sys_MOI;
				final double omega_w = -WINCH_MOTOR.KtNMPerAmp / (WINCH_MOTOR.KvRadPerSecPerVolt * WINCH_MOTOR.rOhms * sys_MOI)
					* (ARM_HOOK_RADIUS / WINCH_SPOOL_RADIUS) * x.get(1, 0);

				this.B_matrix.set(1, 0, WINCH_MOTOR.KtNMPerAmp / (WINCH_MOTOR.rOhms * sys_MOI));

				var xdot = Matrix.mat(Nat.N2(), Nat.N1()).fill(
					x.get(1, 0),	// pass omega to the output
					omega_w - omega_grav - omega_spring
				).plus(this.B_matrix.times(u));
				return xdot;

			}

		}
		public static final class IntegratorV2 implements Sendable {

			protected final double
				ROPE_INITIAL_DELTA = ropeDelta(0.0),
				VEL_EPSILON = 0.1,
				ACC_EPSILON = 10.0,
				TQ_EPSILON = 0.01;

			protected enum State {
				ArmAngle			(0),
				ArmVelocity			(1),
				SpoolDisplacement	(2),
				SpoolVelocity		(3);

				public int idx;
				private State(int i) { this.idx = i; }

				public double from(Matrix<N4, N1> x) {
					return x.get(this.idx, 0);
				}
			}
			protected enum Input {
				Voltage		(0),
				ForwardAcc	(1),
				TimeStep	(2);

				public int idx;
				private Input(int i) { this.idx = i; }

				public double from(Matrix<N3, N1> x) {
					return x.get(this.idx, 0);
				}
			}

			private Matrix<N3, N1> u_input;
			private Matrix<N4, N1> x_state, y_output;

			private double
				current_wrist_angle = 0.0,
				current_arm_RI = getArmRI(0.0);

			public IntegratorV2() {
				this.u_input = VecBuilder.fill(0.0, 0.0, 0.0);
				this.x_state = Matrix.mat(Nat.N4(), Nat.N1()).fill(0.0, 0.0, 0.0, 0.0);
				this.y_output = new Matrix<>(Nat.N4(), Nat.N1());
			}

			public synchronized void applyForwardAcceleration(double robot_acc_fb) {
				this.u_input.set(Input.ForwardAcc.idx, 0, robot_acc_fb);
			}
			public synchronized void setInputs(double winch_volts, double wrist_angle) {
				this.u_input.set(Input.Voltage.idx, 0, winch_volts);
				if(wrist_angle != this.current_wrist_angle) {
					this.current_wrist_angle = wrist_angle;
					this.current_arm_RI = getArmRI(wrist_angle);
				}
			}
			public synchronized void setInputs(double winch_volts, double wrist_angle, double robot_acc_fb) {
				this.setInputs(winch_volts, wrist_angle);
				this.applyForwardAcceleration(robot_acc_fb);
			}
			public void update(double dt_seconds) {
				this.u_input.set(Input.TimeStep.idx, 0, dt_seconds);
				this.x_state = NumericalIntegration.rk4(this::dynamics, this.x_state, this.u_input, dt_seconds);
				this.y_output = this.x_state;
			}

			public double getAngle() {
				return Math.toDegrees(State.ArmAngle.from(this.y_output));
			}
			public double getVelocity() {
				return Math.toDegrees(State.ArmVelocity.from(this.y_output));
			}
			public double getWinchCurrent() {
				return Math.abs(WINCH_MOTOR.getCurrent(
					State.SpoolVelocity.from(this.y_output),
					Input.Voltage.from(this.u_input)
				));
			}
			public boolean getUpperLimit() {
				return this.getAngle() >= MAX_REAL_ARM_ANGLE;
			}


			protected double frictSignum(double vel, double tq) {
				double v = Math.signum(vel < VEL_EPSILON && vel > -VEL_EPSILON ? 0.0 : vel);
				if(v != 0.0) { return -v * ARM_FRICTIONAL_TORQUE; }
				else { return -Math.copySign(Math.min(Math.abs(tq), ARM_FRICTIONAL_TORQUE), tq); }
			}
			protected double applyFrict(double tq, double ftq) {
				if(tq > 0.0) {
					return Math.max(tq + ftq, 0.0);
				} else if(tq < 0.0) {
					return Math.min(tq + ftq, 0.0);
				}
				return 0.0;
			}
			protected Matrix<N4, N1> dynamics(Matrix<N4, N1> x, Matrix<N3, N1> u) {

				final double
					arm_angle = Math.toDegrees(State.ArmAngle.from(x)),
					arm_velocity = State.ArmVelocity.from(x),
					spool_velocity = State.SpoolVelocity.from(x),
					voltage = Input.Voltage.from(u),
					forward_acc = Input.ForwardAcc.from(u),
					dt_seconds = Input.TimeStep.from(u);
				final Translation2d
					arm_CG = getArmCG(arm_angle, this.current_wrist_angle);
				final double
					dx2_rope = ropeDeltaPrimePrime(arm_angle),
					r_pull = getEffectivePullRadius(arm_angle),

					tq_motor = WINCH_MOTOR.getTorque(WINCH_MOTOR.getCurrent(spool_velocity, voltage)),
					tq_grav = getGravitationalTorque(arm_CG),
					tq_spring = getSpringTorque(arm_angle),
					tq_dbase = getForwardAccTorque(forward_acc, arm_CG),
					tq_static = tq_dbase - (tq_grav + tq_spring),
					tq_rope = voltage != 0.0 ? tq_motor / WINCH_SPOOL_RADIUS * r_pull : 0.0,
					tq_friction = frictSignum(arm_velocity, tq_static + tq_rope),
					tq_mfriction = voltage == 0.0 ? Math.copySign(Math.min(Math.abs(tq_motor), Math.abs(tq_static)), tq_motor) : 0.0,
					tq_total = applyFrict(tq_static + tq_rope, tq_friction + tq_mfriction),

					lf_sys = tq_total / r_pull;

				boolean rope_tight = true;
				if(rope_tight) {
					boolean motor_pull = true;
					if(motor_pull) {
						final double
							tq_sys = tq_motor + tq_static;
					} else {
						// subtract arm friction from natural -> total arm torque, then use that for the motor interaction
					}
				} else {

				}

				double
					aa_norm = 0.0,
					av_arm = arm_velocity,
					aa_arm = lf_sys / (
						(dx2_rope * WINCH_FREE_RI / (WINCH_SPOOL_RADIUS * WINCH_SPOOL_RADIUS))
							+ (this.current_arm_RI / r_pull)),
					aa_spool = lf_sys / (
						(WINCH_SPOOL_RADIUS * this.current_arm_RI / (dx2_rope * r_pull))
							+ (WINCH_FREE_RI / WINCH_SPOOL_RADIUS));

				final double TS_EPSILON = Math.toDegrees(arm_velocity * dt_seconds);	// the delta in angle within the next timestep
				if(arm_angle + TS_EPSILON <= MIN_REAL_ARM_ANGLE) {
					aa_norm = -Math.min(aa_arm, 0.0)
						- Math.min(arm_velocity, 0.0) / Math.max(0.004, dt_seconds)		// remove this term for a perfectly ellastic collision
						+ Math.toRadians(Math.abs(MIN_REAL_ARM_ANGLE - arm_angle)) / Math.max(5e-5, 0.5 * dt_seconds * dt_seconds);
					aa_arm = Math.max(aa_norm, aa_arm);
				} else if(arm_angle + TS_EPSILON >= MAX_REAL_ARM_ANGLE) {
					aa_norm = -Math.max(aa_arm, 0.0)
						- Math.max(arm_velocity, 0.0) / Math.max(0.004, dt_seconds)		// ^^
						- Math.toRadians(Math.abs(MAX_REAL_ARM_ANGLE - arm_angle)) / Math.max(5e-5, 0.5 * dt_seconds * dt_seconds);
					aa_arm = Math.min(aa_norm, aa_arm);
				}
				// aa_spool = aa_arm * dx2_rope / WINCH_SPOOL_RADIUS;

				// if(
				// 	av_arm < VEL_EPSILON && av_arm > -VEL_EPSILON &&
				// 	aa_arm < ACC_EPSILON && aa_arm > -ACC_EPSILON &&
				// 	tq_total < TQ_EPSILON && tq_total > -TQ_EPSILON
				// ) {
				// 	av_arm = 0.0;
				// 	aa_arm = 0.0;
				// }

				return Matrix.mat(Nat.N4(), Nat.N1()).fill(
					av_arm,
					aa_arm,
					spool_velocity,
					aa_spool
				);

			}


			@Override
			public void initSendable(SendableBuilder b) {
				b.addDoubleProperty("Arm Angle", this::getAngle, null);
				b.addDoubleProperty("Arm Velocity", this::getVelocity, null);
				b.addDoubleProperty("Arm CG Radius", ()->getArmCG(this.current_wrist_angle).getNorm(), null);
				b.addDoubleProperty("Arm RI", ()->getArmRI(this.current_wrist_angle), null);
				// b.addDoubleProperty("Estimated Spring Torque", ()->estimateSpringTorque(this.getAngle()), null);
				b.addDoubleProperty("Spring Torque", ()->getSpringTorque(this.getAngle()), null);
				b.addDoubleProperty("Gravitational Torque", ()->getGravitationalTorque(this.getAngle(), this.current_wrist_angle), null);
				b.addDoubleProperty("Ext Acceleration Torque", ()->getForwardAccTorque(Input.ForwardAcc.from(this.u_input), this.getAngle(), this.current_wrist_angle), null);
				b.addDoubleProperty("Pull Radius", ()->getEffectivePullRadius(this.getAngle()), null);
				b.addDoubleProperty("Hook to Feed Distance", ()->ropeDelta(this.getAngle()), null);
				b.addDoubleProperty("Rope Delta 2nd Derivative", ()->ropeDeltaPrimePrime(this.getAngle()), null);
				b.addDoubleProperty("Spool Velocity", ()->State.SpoolVelocity.from(this.y_output), null);
				b.addDoubleProperty("Spool Rotations", ()->State.SpoolDisplacement.from(this.y_output) / 2 / Math.PI, null);
			}

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

		public double[] getV1ComponentData() { return Kinematics.getV1ComponentData(this.arm_angle, this.elbow_angle); }
		public double[] getV2ComponentData() { return Kinematics.getV2ComponentData(this.arm_angle, this.elbow_angle); }

		@Override
		public ManipulatorPose interpolate(ManipulatorPose end, double t) {
			return new ManipulatorPose(
				MathUtil.interpolate(this.arm_angle, end.arm_angle, t),
				MathUtil.interpolate(this.elbow_angle, end.elbow_angle, t)
			);
		}

	}
	public static class ManipulatorState implements Interpolatable<ManipulatorState> {

		public final ManipulatorPose pose;
		public double intake_voltage;

		public ManipulatorState(double aa, double ea, double v) {
			this(new ManipulatorPose(aa, ea), v);
		}
		public ManipulatorState(ManipulatorPose p, double v) {
			this.pose = p;
			this.intake_voltage = v;
		}

		@Override
		public ManipulatorState interpolate(ManipulatorState end, double t) {
			return new ManipulatorState(
				this.pose.interpolate(end.pose, t),
				MathUtil.interpolate(this.intake_voltage, end.intake_voltage, t)
			);
		}

	}



	public static final class Arm implements Subsystem, Sendable {

		public static final LimitSwitchSource LMT_FB_SOURCE = LimitSwitchSource.FeedbackConnector;
		public static final LimitSwitchNormal LMT_FB_NORMALITY = LimitSwitchNormal.NormallyOpen;
		public static final FeedbackDevice FB_TYPE = FeedbackDevice.Analog;
		public static final int
			FB_UNITS_PER_RANGE = Constants.ANALOG_UNITS_PER_REVOLUTION,
			CL_IDX = 0;
		public static final boolean
			INVERT_FB_RANGE = false,
			DISABLE_CONINUOUS_FB = true;
		public static final double
			FB_RANGE_DEGREES = 270.0;

		public static double sensorUnitsToDegrees(double units) {
			return (units / FB_UNITS_PER_RANGE) * FB_RANGE_DEGREES;
		}
		public static double degreesToSensorUnits(double degrees) {
			return (degrees / FB_RANGE_DEGREES) * FB_UNITS_PER_RANGE;
		}


		private final WPI_TalonSRX winch;

		private Arm(int id, int x) {
			this.winch = new WPI_TalonSRX(id);

			this.winch.configFactoryDefault();
			this.winch.setNeutralMode(NeutralMode.Brake);
			this.winch.configForwardLimitSwitchSource(LMT_FB_SOURCE, LMT_FB_NORMALITY);
			this.winch.configReverseLimitSwitchSource(LMT_FB_SOURCE, LMT_FB_NORMALITY);
		}
		public Arm(int id) {
			this(id, 0);
			this.initIntegratedSensor();
		}

		private void initIntegratedSensor() {
			this.winch.configSelectedFeedbackSensor(FB_TYPE, CL_IDX, 0);
			this.winch.configFeedbackNotContinuous(DISABLE_CONINUOUS_FB && RobotBase.isReal(), 0);
			this.winch.setSensorPhase(INVERT_FB_RANGE);
			// this.winch.config_kF(CL_IDX, Constants.ARM_ANGLE_KF);
			// this.winch.config_kP(CL_IDX, Constants.ARM_ANGLE_KP);
			// this.winch.config_kI(CL_IDX, Constants.ARM_ANGLE_KI);
			// this.winch.config_kD(CL_IDX, Constants.ARM_ANGLE_KD);
			// this.winch.configMotionAcceleration(
			// 	Constants.ARM_ANGLE_ACC_DEG_PER_SEC_SQRD / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE / 10.0);
			// this.winch.configMotionCruiseVelocity(
			// 	Constants.ARM_ANGLE_CRUISE_DEG_PER_SEC / FB_RANGE_DEGREES * FB_UNITS_PER_RANGE / 10.0);
			// also see: config_IntegralZone(), configClosedLoopPeakOutput(), setStatusFramePeriod(), etc...
		}


		@Override
		public void periodic() {
			// manually handle 'homing' the pot, since the phoenix api doesnt seem to reset analog fb devices :{
		}
		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Arm Raw Units", this::rawSensorPosition, null);
			b.addDoubleProperty("Arm Absolute Degrees", this::getAbsoluteDegrees, null);
			b.addDoubleProperty("Arm Velocity", this::getVelocity, null);
			b.addBooleanProperty("Upper Limit?", ()->this.winch.isFwdLimitSwitchClosed() == 1, null);
			b.addBooleanProperty("Lower Limit?", ()->this.winch.isRevLimitSwitchClosed() == 1, null);
			b.addDoubleProperty("Output Volts", this.winch::getMotorOutputVoltage, null);
			b.addDoubleArrayProperty("Current [In:Out]",
				()->{ return new double[]{
					this.winch.getSupplyCurrent(),
					this.winch.getStatorCurrent()
				}; }, null);
			// b.addDoubleProperty("MC Temp", this.winch::getTemperature, null);
		}


		public void setVoltage(double volts) {
			this.winch.setVoltage(volts);
		}
		public void setTargetPosition(double degrees_absolute) {
			this.winch.set(ControlMode.Position, degreesToSensorUnits(degrees_absolute));
		}
		public void setTargetPosition_MM(double degrees_absolute) {
			this.winch.set(ControlMode.MotionMagic, degreesToSensorUnits(degrees_absolute));
		}

		private double rawSensorPosition() {
			return this.winch.getSelectedSensorPosition(CL_IDX);
		}
		private double rawSensorVelocity() {
			return this.winch.getSelectedSensorVelocity(CL_IDX);
		}

		public double getAbsoluteDegrees() {
			return sensorUnitsToDegrees(this.rawSensorPosition());
		}
		public double getVelocity() {
			return sensorUnitsToDegrees(this.rawSensorVelocity()) * 10.0;
		}


		// public static class ExtSensorArm extends Arm {

		// 	public ExtSensorArm(int id, AnalogPotentiometer sensor) {
				
		// 	}

		// }

	}

	// public static abstract class Wrist implements Subsystem, Sendable {

	// 	public abstract void setDegrees(double degrees_absolute);
	// 	public abstract void setDisabled();

	// 	public abstract double getAbsoluteDegrees();

	// 	@Override
	// 	public void initSendable(SendableBuilder b) {
	// 		b.addDoubleProperty("Wrist Absolute Degrees", this::getAbsoluteDegrees, null);
	// 	}


	// 	public static final class ServoImpl extends Wrist {

	// 		public static final boolean
	// 			INVERT_OUTPUT_RANGE = true && RobotBase.isReal();
	// 		public static final double
	// 			MIN_PULSE_uS = 550.0,
	// 			MAX_PULSE_uS = 2450.0,
	// 			TOTAL_INPUT_RANGE = 270.0,
	// 			GEARING = 3.0 / 2.0,
	// 			TOTAL_OUTPUT_RANGE = TOTAL_INPUT_RANGE / GEARING;

	// 			// PARALLEL_OFFSET_ANGLE = 65.0,
	// 			// STANDARDIZED_MIN_ANGLE = -40.0,
	// 			// STANDARDIZED_MAX_ANGLE = 90.0,
	// 			// PARALLEL_OFFSET_PERCENT = PARALLEL_OFFSET_ANGLE / TOTAL_OUTPUT_RANGE,
	// 			// MIN_PERCENT = (STANDARDIZED_MIN_ANGLE + PARALLEL_OFFSET_ANGLE) / TOTAL_OUTPUT_RANGE,
	// 			// MAX_PERCENT = (STANDARDIZED_MAX_ANGLE + PARALLEL_OFFSET_ANGLE) / TOTAL_OUTPUT_RANGE;

	// 		private final Servo servo;

	// 		public ServoImpl(int pwm_id) {
	// 			this.servo = new Servo(pwm_id);
	// 			this.servo.setBounds(MAX_PULSE_uS / 1000.0, 0.0, 0.0, 0.0, MIN_PULSE_uS / 1000.0);
	// 		}


	// 		@Override
	// 		public void initSendable(SendableBuilder b) {
	// 			super.initSendable(b);
	// 			b.addDoubleProperty("Wrist Percent Range", this::getPercent, null);
	// 			b.addDoubleProperty("Wrist Raw PWM", this.servo::getRaw, null);
	// 		}

	// 		public void setPercent(double percent) {
	// 			percent = Math.max(0.0, Math.min(1.0, percent));
	// 			if(INVERT_OUTPUT_RANGE) { percent = 1.0 - percent; }
	// 			this.servo.setPosition(percent);
	// 		}
	// 		public double getPercent() {
	// 			return this.servo.getPosition();
	// 		}

	// 		@Override
	// 		public void setDegrees(double degrees_absolute) {
	// 			this.setPercent(degrees_absolute / TOTAL_OUTPUT_RANGE);
	// 		}
	// 		@Override
	// 		public void setDisabled() {
	// 			this.servo.setDisabled();
	// 		}
	// 		@Override
	// 		public double getAbsoluteDegrees() {
	// 			return this.getPercent() * TOTAL_OUTPUT_RANGE;
	// 		}


	// 	}
	// 	// public static final class SeatMotorImpl extends Wrist {
	
	// 	// 	private final WPI_TalonSRX main;

	// 	// 	public SeatMotorImpl(int canid) {
	// 	// 		this.main = new WPI_TalonSRX(canid);
	// 	// 		this.main.configFactoryDefault();
	// 	// 	}

	// 	// }
	// }

	public static class Wrist implements Subsystem{
		protected final WPI_TalonSRX wrist_Motor;
		double initialSensorPosition = 0.0;

		public Wrist(int canid){
			this.wrist_Motor = new WPI_TalonSRX(canid);
			this.wrist_Motor.configFactoryDefault();
			initialSensorPosition = wrist_Motor.getSelectedSensorPosition();
		}
		
		public double getEncoderPosition(){
			return wrist_Motor.getSelectedSensorPosition() - initialSensorPosition;
		}

		public void moveWrist(double speed){
			wrist_Motor.set(speed);
		}

		public void moveWristRelativePosition(int encoderDistanceFinal)
		{
			if (getEncoderPosition()!=encoderDistanceFinal)
			{
				moveWrist(-(encoderDistanceFinal-getEncoderPosition())/encoderDistanceFinal);
			}
		}
	}

	public static class WheelIntake implements Subsystem{
		protected final WPI_TalonSRX wheel_Motor;

		public WheelIntake(int canid)
		{
			this.wheel_Motor = new WPI_TalonSRX(canid);
			this.wheel_Motor.configFactoryDefault();
		}

		public void intakeCargo(double speed){
			wheel_Motor.set(speed);
		}
		
	}


	// SET THESE USING VALUES OBTAINED PHYSICALLY OR USING ABSOLUTE SENSOR OUTPUT
	public static final double
		ARM_TOP_LIMIT_TRANSLATION = 102.0,		// physical degrees from top maximum to vertical hang (0.0 deg standard coord)
		ARM_TOP_ABSOLUTE_RAW_POSITION = 625.0,	// the raw position when the arm triggers the upper limit -- transforms to standard coord degrees

		WRIST_PARALLEL_TRANSLATION = 65.0,		// wirst angle when the hand is parallel with the arm -- transforms to standard coord degrees
		WRIST_MIN_ANGLE = -40.0,
		WRIST_MAX_ANGLE = 90.0;					// in standard coord degrees


	public final Arm arm;
	public final Wrist wrist; 
	public final WheelIntake wheelintake;

	private final TalonSRXSimCollection simwinch;
	private final Dynamics.IntegratorV2 simarm = new Dynamics.IntegratorV2();

	private double dynamic_arm_transform = Arm.sensorUnitsToDegrees(ARM_TOP_ABSOLUTE_RAW_POSITION);

	public Manipulator2(Arm a, Wrist w, WheelIntake wi) {
		this.arm = a;
		this.wrist = w;
		this.wheelintake = wi;
		this.simwinch = this.arm.winch.getSimCollection();
	}


	@Override
	public void periodic() {
		this.arm.periodic();
		this.wrist.periodic();
		this.wheelintake.periodic();
		if(RobotBase.isReal() && this.arm.winch.isFwdLimitSwitchClosed() > 0) {
			this.dynamic_arm_transform = this.arm.getAbsoluteDegrees();
		}
	}
	@Override
	public void simulationPeriodic() {
		// nothing.
	}
	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleProperty("Dynamic Transform", ()->this.dynamic_arm_transform, null);
		b.addDoubleProperty("Transformed Arm Angle", this::getArmTransformedAngle, null);
		b.addDoubleProperty("Wrist encoder", this::getWristEncoderValue, null);
		// b.addDoubleProperty("Transformed Wrist Angle", this::getWristTransformedAngle, null);
		// b.addDoubleArrayProperty("Components Pose3d", this::getComponentData, null);
	}

	public void startLogging(String basekey) {
		SmartDashboard.putData(basekey, this);
		SmartDashboard.putData(basekey + "/Arm", this.arm);
		//SmartDashboard.putData(basekey + "/Wrist", this.wrist);
		//SmartDashboard.putData(basekey + "/Wheel Intake", this.wheelintake);
	}
	public void logSim(SenderNT sender) {
		sender.putData("Arm Sim", this.simarm);
	}

	public void simTick(double dt_s) {
		this.simwinch.setBusVoltage(RobotController.getBatteryVoltage());

		this.simarm.setInputs(
			this.simwinch.getMotorOutputLeadVoltage(), 0.0
			/*this.getWristTransformedAngle()*/
		);
		this.simarm.update(dt_s);

		this.simwinch.setAnalogPosition(
			(int)Arm.degreesToSensorUnits( this.arm_toAbsolute(this.simarm.getAngle()) ));
		this.simwinch.setAnalogVelocity(
			(int)Arm.degreesToSensorUnits( this.simarm.getVelocity() / 10.0 ));
		this.simwinch.setLimitFwd(this.simarm.getUpperLimit());
	}


	private double arm_toTransformed(double a) { return a - this.dynamic_arm_transform + ARM_TOP_LIMIT_TRANSLATION; }
	private double arm_toAbsolute(double t) { return t + this.dynamic_arm_transform - ARM_TOP_LIMIT_TRANSLATION; }
	private static double wrist_toTransformed(double a) { return a - WRIST_PARALLEL_TRANSLATION; }
	private static double wrist_toAbsolute(double t) { return t + WRIST_PARALLEL_TRANSLATION; }

	public double getWristEncoderValue()
	{
			return wrist.wrist_Motor.getSelectedSensorPosition();
	}

	public double getArmTransformedAngle() {
		return this.arm_toTransformed(this.arm.getAbsoluteDegrees());
	}

	// public double getWristTransformedAngle() {
	// 	return wrist_toTransformed(this.wrist.getAbsoluteDegrees());
	// }
	// public ManipulatorPose getFeedbackPose() {
	// 	return new ManipulatorPose(
	// 		this.getArmTransformedAngle(),
	// 		this.getWristTransformedAngle()
	// 	);
	// }
	// public double[] getComponentData() {
	// 	return Kinematics.getV1ComponentData(
	// 		this.getArmTransformedAngle(),
	// 		this.getWristTransformedAngle()
	// 	);
	// }

	public void setSimForwardAcc(double robot_acc) {
		this.simarm.applyForwardAcceleration(robot_acc);
	}
	public double getSimCurrentDraw() {
		return this.simarm.getWinchCurrent();
	}

	public void setArmStandardized(double degrees) {
		this.arm.setTargetPosition_MM(this.arm_toAbsolute(degrees));
	}
	// public void setWristStandardized(double degrees) {
	// 	this.wrist.setDegrees(wrist_toAbsolute(degrees));
	// }
	// public void setTargetPose(ManipulatorPose p) {
	// 	this.setArmStandardized(p.arm_angle);
	// 	this.setWristStandardized(p.elbow_angle);
	// }
	// public void setTargetState(ManipulatorState s) {
	// 	this.setTargetPose(s.pose);
	// 	this.hand.setVoltage(s.intake_voltage);
	// }


	// public BaseControl controlManipulatorAdv(
	// 	DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
	// ) {
	// 	return new BaseControl(this, a, g, w);
	// }
	public BaseControl controlManipulatorAdv(
		DoubleSupplier a, DoubleSupplier wup,
		DoubleSupplier wdwn, BooleanSupplier al, BooleanSupplier gl, BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
	) {
		return new BaseControl(this, a, wup, wdwn, al, gl, wheelIntakeRight, wheelIntakeLeft);
	}


	public static class BaseControl extends CommandBase {

		public static final double
			ARM_WINCH_VOLTAGE_SCALE = 5.0,
			ARM_WINCH_LOCK_VOLTS = 1.1,
			GRAB_CLAW_VOLTAGE_SCALE = 12.0,
			GRAB_CLAW_LOCK_VOLTAGE = 8.0,
			WRIST_ACCUMULATION_RATE = 1.8;
		public static final boolean
			ENABLE_BBOX_LIMITING = true;
		
		protected final Manipulator2
			manipulator;
		protected final DoubleSupplier
			arm_rate,
			wrist_up,
			wrist_down;
			// wrist_rate;
		protected final BooleanSupplier
			// wrist_reset,
			arm_lock,
			grab_lock,
			wheel_intake_right,
			wheel_intake_left;

		protected double
			wrist_position = 0.0;
		protected boolean
			arm_locked_state = false,
			intake_locked_state = false;

		// public BaseControl(
		// 	Manipulator2 m,
		// 	DoubleSupplier a, DoubleSupplier g, DoubleSupplier w
		// ) {
		// 	this.manipulator = m;
		// 	this.arm_rate = a;
		// 	this.grab_rate = g;
		// 	this.wrist_rate = w;
		// 	this.wrist_reset = this.arm_lock = this.grab_lock = ()->false;
		// 	wheel_intake_left = false;
		// 	wheel_intake_right = true;
		// 	super.addRequirements(m.arm, m.wrist, m.wheelintake);
		// }
		public BaseControl(
			Manipulator2 m,
			DoubleSupplier a, DoubleSupplier wup,
			DoubleSupplier wdwn, BooleanSupplier al, BooleanSupplier gl,
			BooleanSupplier wheelIntakeRight, BooleanSupplier wheelIntakeLeft
		) {
			this.manipulator = m;
			this.arm_rate = a;
			this.wrist_up = wup;
			this.wrist_down = wdwn;
			this.arm_lock = al;
			this.grab_lock = gl;
			this.wheel_intake_right = wheelIntakeRight;
			this.wheel_intake_left = wheelIntakeLeft;
			super.addRequirements(m.arm, m.wrist, m.wheelintake);
			//this.wrist_rate = w;
			// this.wrist_reset = wr;
		}

		@Override
		public void initialize() {
			this.wrist_position = 0.0;
			this.arm_locked_state = false;
			//this.intake_locked_state = false;
		}
		@Override
		public void execute() {
			// this.wrist_position += this.wrist_rate.getAsDouble() * WRIST_ACCUMULATION_RATE;
			// if(this.wrist_reset.getAsBoolean()) {
			// 	this.wrist_position = 0.0;
			// } else {
			// 	this.wrist_position = Math.min(Math.max(-80.0, this.wrist_position), 100.0);
			// }
			// ^ not complete
			//double lastEncoderPos = Arm.getWristEncoderValue();
			double setWristVal = wrist_up.getAsDouble() - wrist_down.getAsDouble();
			if(wrist_up.getAsDouble() <= 0.02 && wrist_down.getAsDouble() <= 0.02)
			{
				this.manipulator.wrist.moveWrist(0.1);
			}
			else
			{
				this.manipulator.wrist.moveWrist(setWristVal);
			}

			double
				arate = this.arm_rate.getAsDouble();
				// grate = this.grab_rate.getAsDouble();
			if(this.arm_lock.getAsBoolean()) { this.arm_locked_state = !this.arm_locked_state; }
			if(this.grab_lock.getAsBoolean()) { this.intake_locked_state = !this.intake_locked_state; }
			if(arate != 0) { this.arm_locked_state = false; }
			// if(grate != 0) { this.intake_locked_state = false; }
			this.manipulator.arm.setVoltage(this.arm_locked_state ? ARM_WINCH_LOCK_VOLTS : (arate * ARM_WINCH_VOLTAGE_SCALE));
			


			if(wheel_intake_right.getAsBoolean()){
				this.manipulator.wheelintake.intakeCargo(0.8);
				System.out.print("going forwards/right");
			}
			else if (wheel_intake_left.getAsBoolean())
			{
				this.manipulator.wheelintake.intakeCargo(-0.8);
				System.out.print("going backwards/left");
			}
			else{
				this.manipulator.wheelintake.intakeCargo(0.0);
			}
			//this.manipulator.hand.setVoltage(this.intake_locked_state ? GRAB_CLAW_LOCK_VOLTAGE : (grate * GRAB_CLAW_VOLTAGE_SCALE));
			// this.manipulator.setWristStandardized(this.wrist_position);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		public void end(boolean isfinished) {
			this.manipulator.arm.setVoltage(0);
			//this.manipulator.hand.setVoltage(0);
			//this.manipulator.wrist.setDisabled();
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Winch Voltage Setpoint",
				()->this.arm_locked_state ? ARM_WINCH_LOCK_VOLTS : (this.arm_rate.getAsDouble() * ARM_WINCH_VOLTAGE_SCALE), null);
			// b.addDoubleProperty("Grabber Voltage Setpoint",
			// 	()->this.intake_locked_state ? GRAB_CLAW_LOCK_VOLTAGE : (this.grab_rate.getAsDouble() * GRAB_CLAW_VOLTAGE_SCALE), null);
			b.addDoubleProperty("Wrist Position Setpoint", ()->this.wrist_position, null);
			b.addBooleanProperty("Arm Locked", ()->this.arm_locked_state, null);
			b.addBooleanProperty("Grab Locked", ()->this.intake_locked_state, null);
		}

	}
	public class Hand {
	}

}
