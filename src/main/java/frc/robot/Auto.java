package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.team3407.Util;
import frc.robot.team3407.drive.DriveSupplier.*;


public final class Auto {

	private static final SendableChooser<Command> selectable_auto = new SendableChooser<>();
	private static Command option1, option2;
	private static BooleanSupplier hardware_enable, hardware_select;


	public static void setHardwareOptionA(Command a) { option1 = a; }
	public static void setHardwareOptionB(Command b) { option2 = b; }
	public static void setHardwareSelectors(BooleanSupplier enable, BooleanSupplier select) {
		hardware_enable = enable;
		hardware_select = select;
	}
	public static void clearHardwareSelectors() {
		hardware_enable = hardware_select = null;
	}
	public static void addSelectableCommand(String n, Command c) { selectable_auto.addOption(n, c); }

	public static void initialize() {
		selectable_auto.setDefaultOption("No Auto", null);
		SmartDashboard.putData("Autonomous Selector", selectable_auto);
	}
	public static void runSelected() {
		if(hardware_enable != null && hardware_select != null && hardware_enable.getAsBoolean()) {
			System.out.println("Running Hardware Selected Auto...");
			if(hardware_select.getAsBoolean() && option1 != null) {
				option1.schedule();
			} else if(option2 != null) {
				option2.schedule();
			}
		} else {
			System.out.println("Running NT Selected Auto...");
			Command c = selectable_auto.getSelected();
			if(c != null) {
				c.schedule();
			} else {
				System.out.println("No Auto Command Selected!");
			}
		}
	}



	/**
	 * This runs an autonoumus command that drives a given distance at a given velocity.
	 * @param db The dribase subsystem.
	 * @param d Distance in meters.
	 * @param v Velocity in m/s.
	 * @return A command for driving the given distance at the given velocity
	 */
	public static CommandBase driveStraight(DriveBase db, double d, double v) {
		return new DistanceDrive(db, d, v);
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
	/** Get a composed command for driving a distance, then engaging the charging pad - ex. taxi beyond the pad, then climb it
	 * @param db The drivebase subsystem
	 * @param d The distance to taxi
	 * @param v The velocity to taxi at
	 * @param pitch Gyro supplier for the forward incline detection
	 * @param envel engage velocity for climbing
	 * @param icvel incline velocity for climbing
	 * @return The composed commaned
	 */
	public static CommandBase taxiClimb(DriveBase db, double d, double v, Gyro pitch, double envel, double icvel) {
		return driveStraight(db, d, v).andThen(climbPad(db, pitch, envel, icvel));
	}
	/** Get a active parking command - a routine where the robot attempts to stay in the same position using the encoder position and a negative feedback p-loop
	 * @param p_gain The proportional gain in volts/meter that the robot will apply when any position error is accumulated
	 * @return A command for active parking
	 */
	public static ActivePark activePark(DriveBase db, double p_gain) {
		return new ActivePark(db, p_gain);
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

	public static CommandBase setWristAngle(Manipulator m, double a) {
		return new InstantCommand(()->m.grabber.setWristAngle(a));
	}
	public static CommandBase setWristPercent(Manipulator m, double p) {
		return new InstantCommand(()->m.grabber.setWristPercent(p));
	}
	public static AutoGrabControl setGrabber(Manipulator m, double wa, double gv) {
		return new AutoGrabControl(m, wa, gv);
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
			DEFAULT_INCLINE_VELOCITY = 0.15,	// this should be the target, but the db doesn't actually go this speed bc of the angle
			DELTA_ANGLE_THRESH = 14.0,		// the charging pad main incline is 15 degrees, so give some room for error
			DELTA_ANGLE_RATE_THRESH = 15.0,
			CLIMB_DISPLACEMENT_THRESH = 0.8,	// minimum displacement from when incline is first detected to when stabilization can occur
			STABLE_ANGLE_THRESH = 0.5,
			STABLE_ANGLE_RATE_THRESH = 5.0,
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
		private final CommandBase driver;
		private final double engage_velocity, incline_velocity;
		private double
			pitch_init = 0.0,
			climb_init_pos = 0.0,
			start_pos = 0.0,
			fwdvel = 0.0;
		private State
			state = State.STABLE;

		public ClimbPad(DriveBase db, Gyro pa)
			{ this(db, pa, DEFAULT_INCLINE_VELOCITY, DEFAULT_ENGAGE_VELOCITY); }
		public ClimbPad(DriveBase db, Gyro pa, double ev, double iv) {
			this.drivebase = db;
			this.pitch = pa;
			this.driver = db.tankDriveVelocityProfiled(new TankSupplier(()->this.fwdvel, ()->this.fwdvel));
			this.engage_velocity = ev;
			this.incline_velocity = iv;
			super.addRequirements(db);
		}

		private void driveVelocity(double v) {
			this.fwdvel = v;
		}
		private double getAvgWheelPos() {
			return (this.drivebase.getLeftPosition() + this.drivebase.getRightPosition()) / 2.0;
		}
		private boolean tooFar() {
			double pos = this.getAvgWheelPos();
			return (pos - this.start_pos) > 5 || (pos - this.climb_init_pos) > 3;
		}

		@Override
		public void initialize() {
			this.driver.initialize();
			this.state = State.ENGAGING;
			this.pitch_init = this.pitch.getAngle();
			this.start_pos = this.climb_init_pos = this.getAvgWheelPos();
			this.fwdvel = 0.0;
		}
		@Override
		public void execute() {
			switch(this.state) {
				case ENGAGING: {
					this.driveVelocity(this.engage_velocity);
					if(this.pitch_init - this.pitch.getAngle() > DELTA_ANGLE_THRESH) {
						this.state = State.CLIMBING;
						this.climb_init_pos = this.getAvgWheelPos();
					}
					break;
				}
				case CLIMBING: {
					this.driveVelocity(this.incline_velocity);
					if(-this.pitch.getRate() < -DELTA_ANGLE_RATE_THRESH &&	// the actual rate is the inverse bc of CW <--> CCW weridness w/ Gyro interface so invert
						(this.getAvgWheelPos() - this.climb_init_pos) >= CLIMB_DISPLACEMENT_THRESH
					) {
						this.state = State.STABILIZING;
						// maybe store the position here, use position locking in the stabilization block?
					}
					break;
				}
				case STABILIZING: {
					this.driveVelocity(0.0);	// or use position lock
					double da = this.pitch_init - this.pitch.getAngle();
					if(Math.abs(da) < STABLE_ANGLE_THRESH && Math.abs(this.pitch.getRate()) < STABLE_ANGLE_RATE_THRESH &&
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
					this.driveVelocity(-this.incline_velocity);
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
			this.driver.execute();
		}
		@Override
		public boolean isFinished() {
			return this.state == State.STABLE || this.tooFar();
		}
		@Override
		public void end(boolean i) {
			this.drivebase.setDriveVoltage(0, 0);
			this.driver.end(i);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addStringProperty("Control State", ()->this.state.desc, null);
			b.addDoubleProperty("Velocity Setpoint", ()->this.fwdvel, null);
		}

	}

	public static class DistanceDrive extends CommandBase {

		public static final double STOP_BUFF_VEL_SCALE = 0.05;	// the amount of stopping buffer should scale with the velocity driven at

		private final DriveBase drivebase;
		private final CommandBase driver;
		private final double target, velocity;
		private double init = 0.0;

		public DistanceDrive(DriveBase db, double x, double v) {
			this.drivebase = db;
			this.driver = db.tankDriveVelocityProfiled(StaticSupplier.genSimple(v));
			this.target = x;
			this.velocity = v;
		}

		public double posAvg() {
			return (this.drivebase.getLeftPosition() + this.drivebase.getRightPosition()) / 2.0;
		}

		@Override
		public void initialize() {
			this.driver.initialize();
			this.init = posAvg();
		}
		@Override
		public void execute() {
			this.driver.execute();
		}
		@Override
		public boolean isFinished() {
			return Math.abs(this.posAvg() - this.init) >= Math.abs(this.target - this.velocity * STOP_BUFF_VEL_SCALE);
		}
		@Override
		public void end(boolean i) {
			this.driver.end(i);
		}

	}

	public static class AutoGrabControl extends CommandBase {

		private final Manipulator.Grabber grabber;
		private final double wrist_angle, grab_volts;

		public AutoGrabControl(Manipulator m, double wangle, double gvolts) {
			grabber = m.grabber;
			this.wrist_angle = wangle;
			this.grab_volts = gvolts;
		}

		@Override
		public void initialize() {
			this.grabber.setWristAngle(this.wrist_angle);
		}
		@Override
		public void execute() {
			this.grabber.setGrabberVoltage(this.grab_volts);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		@Override
		public void end(boolean i) {
			
		}

	}
	public static class AutoArmControl extends CommandBase {

		private final Manipulator.Arm arm;
		private final double voltage;

		public AutoArmControl(Manipulator m, double volts) {
			this.arm = m.arm;
			this.voltage = volts;
		}

		@Override
		public void initialize() {}
		@Override
		public void execute() {
			this.arm.setWinchVoltage(this.voltage);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		@Override
		public void end(boolean i) {
			
		}

	}
	

}
