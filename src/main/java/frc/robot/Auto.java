package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.team3407.drive.DriveSupplier.StaticSupplier;


public class Auto {

	/**
	 * This runs an autonoumus command that drives a given distance at a given velocity.
	 * @param db The dribase subsystem.
	 * @param d Distance in meters.
	 * @param v Velocity in m/s.
	 * @return A command for driving the given distance at the given velocity
	 */
	public static CommandBase driveStraight(DriveBase db, double d, double v) {
		return new DriveBase.TankDriveVelocityProfiled(db, StaticSupplier.genSimple(v)).withTimeout(d / v);
	}
	/** Get a active parking command - a routine where the robot attempts to stay in the same position using the encoder position and a negative feedback p-loop
	 * @param p_gain The proportional gain in volts/meter that the robot will apply when any position error is accumulated
	 * @return A command for active parking
	 */
	public static ActivePark activePark(DriveBase db, double p_gain) {
		return new ActivePark(db, p_gain);
	}
	/** Get a command for driving up the charging pad and balancing at the top.
	 * @param db The drivebase subsystem
	 * @param pitch	A gyro implementation for the robot's pitch axis
	 * @param vel The velocity to climb the pad at
	 * @return A runnable command
	 */
	public static ClimbPad climbPad(DriveBase db, Gyro pitch, double evel, double ivel) {
		return new ClimbPad(db, pitch, evel, ivel);
	}
	/**	Get a command for parking on the charging pad based on the gyro angle
	 * @param db The drivebase subsystem
	 * @param pitch A gyro implementation for the robot's pitch axis
	 * @param kp The p-gain value - VOLTS PER DEGREE [of error]
	 * @return The command
	 */
	public static BalancePark balancePark(DriveBase db, Gyro pitch, double kp) {
		return new BalancePark(db, pitch, kp);
	}




	public static class ActivePark extends CommandBase {

		private final DriveBase drivebase;
		private final double volts_per_meter;
		private double linit, rinit;

		public ActivePark(DriveBase db, double p) { // p is the proportional gain, in volts per meter [error]
			this.drivebase = db;
			this.volts_per_meter = p;
		}

		@Override
		public void initialize() {
			this.linit = this.drivebase.getLeftPosition();
			this.rinit = this.drivebase.getRightPosition();
		}
		@Override
		public void execute() {
			double le = this.drivebase.getLeftPosition() - this.linit;
			double re = this.drivebase.getRightPosition() - this.rinit;
			this.drivebase.setDriveVoltage(
				-le * this.volts_per_meter,     // we are assuming that positive position for the encoders is the same direction as positive voltage
				-re * this.volts_per_meter
			);
		}
		@Override
		public void end(boolean interrupted) {
			this.drivebase.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}

	}
	public static class BalancePark extends CommandBase {

		private final DriveBase drivebase;
		private final Gyro pitch_axis;
		private final double volts_per_degree;
		private double
			left_target, right_target,
			pitch_target;

		public BalancePark(DriveBase db, Gyro pitch, double kp_v) {
			this.drivebase = db;
			this.pitch_axis = pitch;
			this.volts_per_degree = kp_v;
		}

		@Override
		public void initialize() {
			this.left_target = this.drivebase.getLeftPosition();
			this.right_target = this.drivebase.getRightPosition();
			this.pitch_target = this.pitch_axis.getAngle();
		}
		@Override
		public void execute() {
			double ae = this.pitch_target - this.pitch_axis.getAngle();
			this.drivebase.setDriveVoltage(
				ae * this.volts_per_degree,
				ae * this.volts_per_degree
			);
		}
		@Override
		public void end(boolean i) {
			this.drivebase.setDriveVoltage(0, 0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}

	}
	public static class ClimbPad extends CommandBase {

		public static final double
			DEFAULT_ENGAGE_VELOCITY = 0.8,
			DEFAULT_INCLINE_VELOCITY = 0.1,	// this should be the target, but the db doesn't actually go this speed bc of the angle
			DELTA_ANGLE_THRESH = 14.0,	// the charging pad main incline is 15 degrees, so give some room for error
			DELTA_ANGLE_RATE_THRESH = 20,
			STABLE_ANGLE_THRESH = 0.5,
			STABLE_VELOCITY_THRESH = 0.03;

		private static enum State {
			ENGAGING	("Engaging"),
			CLIMBING	("Climbing"),
			STABILIZING	("Stabilizing"),
			OVERSHOT	("Overshot"),
			STABLE		("Stablized");

			public final String desc;
			private State(String s) { this.desc = s; }
		}

		private final DriveBase drivebase;
		private final Gyro pitch;
		private final PIDController left_fb, right_fb;
		private final double engage_velocity, incline_velocity;
		private double pitch_init = 0.0;
		private State state = State.STABLE;

		public ClimbPad(DriveBase db, Gyro pa)
			{ this(db, pa, DEFAULT_INCLINE_VELOCITY, DEFAULT_ENGAGE_VELOCITY); }
		public ClimbPad(DriveBase db, Gyro pa, double ev, double iv) {
			this.drivebase = db;
			this.pitch = pa;
			this.left_fb = drivebase.parameters.getFeedbackController();
			this.right_fb = drivebase.parameters.getFeedbackController();
			this.engage_velocity = ev;
			this.incline_velocity = iv;
			super.addRequirements(db);
		}

		private void driveVelocity(double lv, double rv) {
			double
				lc = this.drivebase.getLeftVelocity(),  // the actual velocity
				rc = this.drivebase.getRightVelocity();
			this.drivebase.setDriveVoltage(
				this.drivebase.feedforward.calculate(lv) +  // the calculated feedforward
					this.left_fb.calculate(lc, lv),   		// add the feedback adjustment
				this.drivebase.feedforward.calculate(rv) +
					this.right_fb.calculate(rc, rv)
			);
		}

		@Override
		public void initialize() {
			this.left_fb.reset();
			this.right_fb.reset();
			this.state = State.ENGAGING;
			this.pitch_init = this.pitch.getAngle();
		}
		@Override
		public void execute() {
			switch(this.state) {
				case ENGAGING: {
					this.driveVelocity(
						this.engage_velocity,
						this.engage_velocity
					);
					if(pitch_init - this.pitch.getAngle() > DELTA_ANGLE_THRESH) {
						this.state = State.CLIMBING;
					}
					break;
				}
				case CLIMBING: {
					this.driveVelocity(
						this.incline_velocity,
						this.incline_velocity
					);
					if(-this.pitch.getRate() < -DELTA_ANGLE_RATE_THRESH) {	// the actual rate is the inverse bc of CW <--> CCW weridness w/ Gyro interface so invert
						this.state = State.STABILIZING;
						// maybe store the position here, use position locking in the stabilization block?
					}
					break;
				}
				case STABILIZING: {
					this.driveVelocity(0.0, 0.0);	// or use position lock
					double da = this.pitch_init - this.pitch.getAngle();
					if(Math.abs(da) < STABLE_ANGLE_THRESH &&
						Math.abs(this.drivebase.getLeftVelocity()) < STABLE_VELOCITY_THRESH &&
						Math.abs(this.drivebase.getRightVelocity()) < STABLE_VELOCITY_THRESH)
					{
						this.state = State.STABLE;
					} else if(da < -DELTA_ANGLE_THRESH) {
						this.state = State.OVERSHOT;
					} else if(da > DELTA_ANGLE_THRESH) {
						this.state = State.CLIMBING;
					}
					break;
				}
				case OVERSHOT: {
					this.driveVelocity(
						-this.incline_velocity,
						-this.incline_velocity
					);
					if(-this.pitch.getRate() > DELTA_ANGLE_RATE_THRESH) {	// see above for inverting rate
						this.state = State.STABILIZING;
					}
					break;
				}
				default:
				case STABLE: {
					break;
				}
			}
		}
		@Override
		public boolean isFinished() {
			return this.state == State.STABLE;
		}
		@Override
		public void end(boolean i) {
			this.drivebase.setDriveVoltage(0, 0);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addStringProperty("Control State", ()->this.state.desc, null);
		}

	}

}
