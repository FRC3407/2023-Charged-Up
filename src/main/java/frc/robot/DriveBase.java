package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.robot.team3407.drive.Types.*;
import frc.robot.team3407.drive.DriveSupplier.*;


public final class DriveBase extends MotorSafety implements Subsystem, Sendable {

	public static class ClosedLoopParams {
		public final Inversions
			encoder_inversions;
		public final double
			track_width_meters,
			wheel_diameter_meters,

			static_voltage,						// kS -- drive motor feedforward term -- "voltage to overcome static friction" -- volts
			volt_seconds_per_meter,				// kV -- drive motor feedforward term -- "voltage to hold a given velocity" -- volts * seconds / meters
			volt_seconds_sqrd_per_meter,		// kA -- drive motor feedforward term -- "voltage to hold an acceleration" -- volts * seconds^2 / meters
			volt_seconds_per_meter_gain,		// kP -- proportional feedback gain -- "voltage applied per unit velocity error" -- volts * seconds / meters
			volts_per_meter_gain,				// kI -- integral feedback gain -- "voltage applied per unit displacement error" -- volts / meters
			volt_seconds_sqrd_per_meter_gain,	// kD -- derivitive feedback gain -- "voltage applied per unit acceleration error" -- volts * seconds^2 / meters

			ramsete_B,
			ramsete_Z,

			max_voltage_output,				// maximum voltage to be applied to the drivebase motors -- volts
			max_velocity,					// maximum velocity of either side -- meters / second
			max_acceleration,				// maximum acceleration of either side -- meters / second^2
			max_jerk						// maximum jerk of either side for profiled velocity drive -- meters / second^3
		;
		public ClosedLoopParams(
			double trackwidth, double wheeldiameter,
			double kS, double kV, double kA,
			double kP, double kI, double kD,
			double ramsete_B, double ramsete_Z,
			double max_volts, double max_vel,
			double max_acc, double max_jrk,
			Inversions encoderinversions
		) {
			this.encoder_inversions = encoderinversions;
			this.track_width_meters = trackwidth;
			this.wheel_diameter_meters = wheeldiameter;
			this.static_voltage = kS;
			this.volt_seconds_per_meter = kV;
			this.volt_seconds_sqrd_per_meter = kA;
			this.volt_seconds_per_meter_gain = kP;
			this.volts_per_meter_gain = kI;
			this.volt_seconds_sqrd_per_meter_gain = kD;
			this.ramsete_B = ramsete_B;
			this.ramsete_Z = ramsete_Z;
			this.max_voltage_output = max_volts;
			this.max_velocity = max_vel;
			this.max_jerk = max_jrk;
			this.max_acceleration = max_acc;
		}
		public double kS() { return this.static_voltage; }
		public double kV() { return this.volt_seconds_per_meter; }
		public double kA() { return this.volt_seconds_sqrd_per_meter; }
		public double kP() { return this.volt_seconds_per_meter_gain; }
		public double kI() { return this.volts_per_meter_gain; }
		public double kD() { return this.volt_seconds_sqrd_per_meter_gain; }
		public PIDConstants kPID() { return new PIDConstants(this.kP(), this.kI(), this.kD()); }
		public SimpleMotorFeedforward getFeedforward() {
			return new SimpleMotorFeedforward(
				this.kS(),
				this.kV(),
				this.kA()
			);
		}
		public PIDController getFeedbackController() {	// should probably be a 'generator' rather than a 'getter'
			return new PIDController(this.kP(), this.kI(), this.kD());
		}
		public ProfiledPIDController getPosProfiledFeedbackController() {	// get a profiled controller for use with POSITIONS
			return new ProfiledPIDController(
				this.kP(), this.kI(), this.kD(),
				new TrapezoidProfile.Constraints(this.max_velocity, this.max_acceleration)
			);
		}
		public ProfiledPIDController getVelProfiledFeedbackController() {	// get a profiled controller for use with VELOCITIES
			return new ProfiledPIDController(
				this.kP(), this.kI(), this.kD(),
				new TrapezoidProfile.Constraints(this.max_acceleration, this.max_jerk)
			);
		}
		public RamseteController getRamseteController() {
			return new RamseteController(
				this.ramsete_B, this.ramsete_Z
			);
		}
		public PathConstraints getMaxConstraints() {	// not recommended for use w/ paths
			return new PathConstraints(this.max_velocity, this.max_acceleration);
		}
	}


	private final DifferentialDriveKinematics kinematics;
	private final DifferentialDriveOdometry odometry;
	public final SimpleMotorFeedforward feedforward;
	public final ClosedLoopParams parameters;

	private final WPI_TalonSRX left, left2, right, right2;
	private final Gyro gyro;

	private Pose2d last_total_odometry = new Pose2d();


	public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params)
		{ this(map, gy, params, Constants.DRIVEBASE_NEUTRAL_MODE); }
	public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params, NeutralMode nmode) {
		this.parameters = params;
		this.gyro = gy;
		this.left = map.front_left;
		this.right = map.front_right;
		this.left2 = map.back_left;
		this.right2 = map.back_right;

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d(), 0, 0);
		this.kinematics = new DifferentialDriveKinematics(this.parameters.track_width_meters);
		this.feedforward = this.parameters.getFeedforward();

		this.left.configFactoryDefault();
		this.right.configFactoryDefault();
		this.left2.configFactoryDefault();
		this.right2.configFactoryDefault();
		this.setNeutralMode(nmode);
		this.left2.follow(this.left);
		this.right2.follow(this.right);
		this.left2.setInverted(InvertType.FollowMaster);
		this.right2.setInverted(InvertType.FollowMaster);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
		this.left.setSensorPhase(parameters.encoder_inversions.left);
		this.right.setSensorPhase(parameters.encoder_inversions.right);
	}





	/* TELEMETRY */

	@Override
	public void periodic() {
		this.odometry.update(
			this.gyro.getRotation2d(),
			this.getLeftPosition(),
			this.getRightPosition()
		);
	}
	@Override
	public void initSendable(SendableBuilder b) {
		b.setSmartDashboardType("DriveBase [4x TalonSRX Differential]");
		b.addDoubleProperty("Left Distance", this::getLeftPosition, null);
		b.addDoubleProperty("Right Distance", this::getRightPosition, null);
		b.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
		b.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
		b.addDoubleProperty("Rotation (Total)", this::getContinuousAngle, null);
		b.addDoubleArrayProperty("Output Voltage [L1, R1, L2, R2]",
			()->{ return new double[]{
				this.left.getMotorOutputVoltage(),
				this.right.getMotorOutputVoltage(),
				this.left2.getMotorOutputVoltage(),
				this.right2.getMotorOutputVoltage()
			}; }, null);
		b.addDoubleArrayProperty("Input Current [L1, R1, L2, R2]",
			()->{ return new double[]{
				this.left.getSupplyCurrent(),
				this.right.getSupplyCurrent(),
				this.left2.getSupplyCurrent(),
				this.right2.getSupplyCurrent()
			}; }, null);
		b.addDoubleArrayProperty("Output Current [L1, R1, L2, R2]",
			()->{ return new double[]{
				this.left.getStatorCurrent(),
				this.right.getStatorCurrent(),
				this.left2.getStatorCurrent(),
				this.right2.getStatorCurrent()
			}; }, null);
		b.addDoubleArrayProperty("Controller Temps [L1, R1, L2, R2]",
			()->{ return new double[]{
				this.left.getTemperature(),
				this.right.getTemperature(),
				this.left2.getTemperature(),
				this.right2.getTemperature()
			}; }, null);
		b.addDoubleArrayProperty("Odometry Tracked Pose [Absolute, Delta]",
			()->{
				Pose2d total = this.getTotalPose();
				Pose2d delta = this.getDeltaPose();
				return new double[]{
					total.getX(), total.getY(), total.getRotation().getDegrees(),
					delta.getX(), delta.getY(), delta.getRotation().getDegrees()
				};
			}, null);
	}
	@Override
	public void stopMotor() {
		this.left.stopMotor();
		this.left2.stopMotor();
		this.right.stopMotor();
		this.right2.stopMotor();
		super.feed();
	}
	@Override
	public String getDescription() {
		return "4xTalonSRX Differential Drivebase";
	}

	public void setTotalFieldPose(FieldObject2d fo) {
		fo.setPose(this.getTotalPose());
	}
	public void setDeltaFieldPose(FieldObject2d fo) {
		fo.setPose(this.getDeltaPose());
	}





	/* COMMAND GENERATORS */

	/** Get a tankdrive command, controlled by inputs that return input percentages (ei. joystick axis outputs: -1 to 1 )
	 * @param ds a drive supplier that supplies percentages in the range [-1, 1]
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDrivePercent(DifferentialDriveSupplier ds) {
		return new TankDrivePercent(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that return voltages for each side
	 * @param ds a drive supplier that supplies voltages in the range [-12, 12]
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVoltage(DifferentialDriveSupplier ds) {
		return new TankDriveVoltage(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that provide target velocities for each side
	 * @param ds a drive supplier that supplies velocities in METERS PER SECOND
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVelocity(DifferentialDriveSupplier ds) {
		return new TankDriveVelocity(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that provide target velocities for each side -- transitions are limited by a trapazoid profile generator
	 * @param ds a drive supplier that supplies velocities in METERS PER SECOND
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVelocityProfiled(DifferentialDriveSupplier ds) {
		return new TankDriveVelocityProfiled(this, ds);
	}
	/** Follow a generic trajectory using WPILib's RamseteCommand ("basic following") */
	public PathFollower followTrajectory(Trajectory t) {
		return new PathFollower(this, t);
	}
	/** Follow a path planner trajectory using the target's filename and executed by a PPRamseteCommand */
	public PathFollower followPPTrajectory(String ppfile) {
		return new PathFollower(this, ppfile);
	}
	/** Follow a path planner trajectory, executed by a PPRamseteCommand */
	public PathFollower followPPTrajectory(PathPlannerTrajectory t) {
		return new PathFollower(this, t);
	}
	/** Follow a path planner trajectory using the target's filename and executed by a PPRamseteCommand */
	public PathFollower followPPTrajectory(String ppfile, DriverStation.Alliance alliance_transform) {
		return new PathFollower(this, ppfile, alliance_transform);
	}
	/** Follow a path planner trajectory, executed by a PPRamseteCommand */
	public PathFollower followPPTrajectory(PathPlannerTrajectory t, DriverStation.Alliance alliance_transform) {
		return new PathFollower(this, t, alliance_transform);
	}
	/** Follow a path planner trajectory with events using the target's filename -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
	public PathFollower followEventTrajectory(String ppfile, Map<String, Command> events) {
		return new PathFollower(this, ppfile, events);
	}
	/** Follow a path planner trajectory with events -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
	public PathFollower followEventTrajectory(PathPlannerTrajectory t, Map<String, Command> events) {
		return new PathFollower(this, t, events);
	}
	/** Follow a path planner trajectory with events using the target's filename -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
	public PathFollower followEventTrajectory(String ppfile, Map<String, Command> events, DriverStation.Alliance alliance_transform) {
		return new PathFollower(this, ppfile, events, alliance_transform);
	}
	/** Follow a path planner trajectory with events -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
	public PathFollower followEventTrajectory(PathPlannerTrajectory t, Map<String, Command> events, DriverStation.Alliance alliance_transform) {
		return new PathFollower(this, t, events, alliance_transform);
	}





	/* SETTERS */

	public void setDriveVoltage(double lv, double rv) {
		this.left.setVoltage(lv);
		this.right.setVoltage(rv);
		super.feed();
	}
	public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
	public void zeroHeading() {
		this.gyro.reset();
	}
	public void resetOdometry(Pose2d p) {
		this.resetEncoders();
		this.last_total_odometry = this.getTotalPose();
		this.odometry.resetPosition(
			this.getRotation(), 0.0, 0.0, p);
	}

	public void setNeutralMode(NeutralMode m) {
		this.left.setNeutralMode(m);
		this.left2.setNeutralMode(m);
		this.right.setNeutralMode(m);
		this.right2.setNeutralMode(m);
	}
	public void setCoastMode() { this.setNeutralMode(NeutralMode.Coast); }
	public void setBrakeMode() { this.setNeutralMode(NeutralMode.Brake); }





	/* GETTERS */

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
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());
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


	public Pose2d getDeltaPose() {
		return this.odometry.getPoseMeters();
	}
	public Pose2d getTotalPose() {
		Pose2d n = this.getDeltaPose();
		return this.last_total_odometry.plus(new Transform2d(n.getTranslation(), n.getRotation()));
	}

	public DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return new DifferentialDriveVoltageConstraint(
			this.feedforward, this.kinematics, this.parameters.max_voltage_output
		);
	}
	public TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(
			this.parameters.max_velocity,
			this.parameters.max_acceleration
		).setKinematics(
			this.kinematics
		).addConstraint(
			this.getVoltageConstraint()
		);
	}

	public RamseteAutoBuilder makeAutoBuilder(HashMap<String, Command> events) {
		return new RamseteAutoBuilder(
			this::getDeltaPose,
			this::resetOdometry,
			this.parameters.getRamseteController(),
			this.kinematics,
			this.feedforward,
			this::getWheelSpeeds,
			this.parameters.kPID(),
			this::setDriveVoltage,
			events,
			this
		);
	}








	/* COMMANDS */

	/** TankDrive that is controlled by 2 'voltage-returning' analog suppliers */
	public static class TankDriveVoltage extends CommandBase {
		
		private final DriveBase drivebase;
		private final DifferentialDriveSupplier driver;

		public TankDriveVoltage(DriveBase db, DoubleSupplier l, DoubleSupplier r)
			{ this(db, new TankSupplier(l, r)); }
		public TankDriveVoltage(DriveBase db, DifferentialDriveSupplier ds) {
			this.drivebase = db;
			this.driver = ds;
			super.addRequirements(db);
		}

		@Override
		public void initialize() {

		}
		@Override
		public void execute() {
			this.drivebase.setDriveVoltage(
				this.driver.leftOutput(),
				this.driver.rightOutput()
			);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		@Override
		public void end(boolean interrupted) {
			this.drivebase.setDriveVoltage(0, 0);
		}

	}



	/** TankDrive that is controlled by 2 'percent-returning' analog suppliers -- ex. joystick axis */
	public static class TankDrivePercent extends TankDriveVoltage {

		public TankDrivePercent(DriveBase db, DoubleSupplier l, DoubleSupplier r) { super(db, l, r); }
		public TankDrivePercent(DriveBase db, DifferentialDriveSupplier ds) { super(db, ds); }

		@Override
		public void execute() {
			super.drivebase.setDriveVoltage(
				super.driver.leftOutput() * super.drivebase.parameters.max_voltage_output,
				super.driver.rightOutput() * super.drivebase.parameters.max_voltage_output
			);
		}

	}





	/** TankDrive that is controlled by 2 'velocity-returning' analog suppliers -- **METERS PER SECOND** */
	public static class TankDriveVelocity extends CommandBase {

		private final DriveBase drivebase;
		private final DifferentialDriveSupplier driver;
		private final PIDController leftfb, rightfb;

		private DifferentialDriveSupplier.CombinedOutput
			outputs = new DifferentialDriveSupplier.CombinedOutput();
		private double la, ra;

		public TankDriveVelocity(
			DriveBase db, DifferentialDriveSupplier ds
		) {
			this.drivebase = db;
			this.driver = ds;
			this.leftfb = db.parameters.getFeedbackController();
			this.rightfb = db.parameters.getFeedbackController();
			super.addRequirements(db);
		}

		@Override
		public void initialize() {
			this.leftfb.reset();
			this.rightfb.reset();
			this.outputs.left = this.drivebase.getLeftVelocity();
			this.outputs.right = this.drivebase.getRightVelocity();
		}
		@Override
		public void execute() {
			double
				lastlt = this.outputs.left,
				lastrt = this.outputs.right;
			this.driver.getOutputs(this.outputs);
			this.la = (this.outputs.left - lastlt) / 0.02;
			this.ra = (this.outputs.right - lastrt) / 0.02;
			this.drivebase.setDriveVoltage(
				this.drivebase.feedforward.calculate(this.outputs.left, la) +
					this.leftfb.calculate(this.drivebase.getLeftVelocity(), this.outputs.left),
				this.drivebase.feedforward.calculate(this.outputs.right, ra) +
					this.rightfb.calculate(this.drivebase.getRightVelocity(), this.outputs.right)
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

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Left Target MpS", ()->this.outputs.left, null);
			b.addDoubleProperty("Right Target MpS", ()->this.outputs.right, null);
			b.addDoubleProperty("Left Acceleration MpS^2", ()->this.la, null);
			b.addDoubleProperty("Right Acceleration MpS^2", ()->this.ra, null);
		}

	}




	/* TankDrive that is controlled by velocity setpoint suppliers and limited to the drivebase's max acceleration/jerk */
	public static class TankDriveVelocityProfiled extends CommandBase {

		private final DriveBase drivebase;
		private final DifferentialDriveSupplier driver;
		private final ProfiledPIDController leftfb, rightfb;

		private DifferentialDriveSupplier.CombinedOutput
			outputs = new DifferentialDriveSupplier.CombinedOutput();
		private double la, ra;

		public TankDriveVelocityProfiled(
			DriveBase db, DifferentialDriveSupplier ds
		) {
			this.drivebase = db;
			this.driver = ds;
			this.leftfb = db.parameters.getVelProfiledFeedbackController();
			this.rightfb = db.parameters.getVelProfiledFeedbackController();
			super.addRequirements(db);
		}

		@Override
		public void initialize() {
			this.leftfb.reset(0);
			this.rightfb.reset(0);
			this.outputs.left = this.drivebase.getLeftVelocity();
			this.outputs.right = this.drivebase.getRightVelocity();
		}
		@Override
		public void execute() {
			double
				lastls = this.outputs.left,
				lastrs = this.outputs.right;
			this.driver.getOutputs(this.outputs);
			double
				lfb = this.leftfb.calculate(this.drivebase.getLeftVelocity(), this.outputs.left),
				rfb = this.rightfb.calculate(this.drivebase.getRightVelocity(), this.outputs.right);
			this.outputs.left = this.leftfb.getSetpoint().position;
			this.outputs.right = this.rightfb.getSetpoint().position;
			this.la = (this.outputs.left - lastls) / 0.02;
			this.ra = (this.outputs.right - lastrs) / 0.02;
			this.drivebase.setDriveVoltage(
				this.drivebase.feedforward.calculate(this.outputs.left, this.la) + lfb,	// feedfoward calculated from the setpoint, plus the feecback
				this.drivebase.feedforward.calculate(this.outputs.right, this.ra) + rfb	// ^
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

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Left Setpoint MpS", ()->this.outputs.left, null);
			b.addDoubleProperty("Right Setpoint MpS", ()->this.outputs.right, null);
			b.addDoubleProperty("Left Acceleration MpS^2", ()->this.la, null);
			b.addDoubleProperty("Right Acceleration MpS^2", ()->this.ra, null);
		}

	}





	/* TRAJECTORY */
	public static final PathConstraints DEFAULT_FOLLOW_CONSTRAINTS =
		new PathConstraints(Constants.TRAJECTORY_MAX_VEL, Constants.TRAJECTORY_MAX_ACC);

	public static PathPlannerTrajectory loadFileConstrainedTrajectory(String ppfile, PathConstraints dfault) {
		PathConstraints p = PathPlanner.getConstraintsFromPath(ppfile);
		return PathPlanner.loadPath(ppfile, (p == null ? dfault : p));
	}
	public static PathPlannerTrajectory loadFileConstrainedTrajectory(String ppfile) {
		return loadFileConstrainedTrajectory(ppfile, DEFAULT_FOLLOW_CONSTRAINTS);
	}
	public static PathPlannerTrajectory transformForActiveAlliance(PathPlannerTrajectory t) {
		return PathPlannerTrajectory.transformTrajectoryForAlliance(t, DriverStation.getAlliance());
	}

	public RamseteCommand followTrajectoryBase(Trajectory t) {
		return new RamseteCommand(
			t,
			this::getDeltaPose,
			this.parameters.getRamseteController(),
			this.feedforward,
			this.kinematics,
			this::getWheelSpeeds,
			this.parameters.getFeedbackController(),
			this.parameters.getFeedbackController(),
			this::setDriveVoltage,
			this
		);
	}
	public PPRamseteCommand followPPTrajectoryBase(PathPlannerTrajectory t, boolean alliance_transform) {
		return new PPRamseteCommand(
			t,
			this::getDeltaPose,
			this.parameters.getRamseteController(),
			this.feedforward,
			this.kinematics,
			this::getWheelSpeeds,
			this.parameters.getFeedbackController(),
			this.parameters.getFeedbackController(),
			this::setDriveVoltage,
			alliance_transform,
			this
		);
	}
	public FollowPathWithEvents followEventTrajectoryBase(PathPlannerTrajectory t, Map<String, Command> events, boolean alliance_transform) {
		return new FollowPathWithEvents(
			this.followPPTrajectoryBase(t, alliance_transform),
			t.getMarkers(),
			events
		);
	}

	/** This class acts as an abstraction layer; combining basic, pathplanner,
	 * and pathplanner w/ event based trajectory following methods into a single class.
	 * The class also provides a simple way to set whether or not the trajectory should
	 * be run 'relatively', and if the drivebase should be manually stopped when the
	 * trajectory finishes. */
	public static class PathFollower extends CommandBase {

		public static final DriverStation.Alliance
			DEFAULT_TRAJECTORY_ALLIANCE = DriverStation.Alliance.Invalid;	// use DriverStation.getAlliance() for active
		public static final boolean
			DEFAULT_FOLLOW_RELATIVE = true,
			DEFAULT_STOP_ON_FINISH = true;

		protected final CommandBase follower;
		protected final DriveBase drivebase;
		protected final Pose2d initial;
		protected boolean
			relative = DEFAULT_FOLLOW_RELATIVE,
			stop_finish = DEFAULT_STOP_ON_FINISH;


		/** Follow a generic trajectory using WPILib's RamseteCommand ("basic following") */
		public PathFollower(DriveBase db, Trajectory t) {
			this.follower = db.followTrajectoryBase(t);
			this.drivebase = db;
			this.initial = t.getInitialPose();
		}
		/** Follow a path planner trajectory using the target's filename and executed by a PPRamseteCommand */
		public PathFollower(DriveBase db, String ppfile)
			{ this(db, loadFileConstrainedTrajectory(ppfile)); }
		/** Follow a path planner trajectory, executed by a PPRamseteCommand */
		public PathFollower(DriveBase db, PathPlannerTrajectory t)
			{ this(db, t, DEFAULT_TRAJECTORY_ALLIANCE); }
		/** Follow a path planner trajectory using the target's filename and executed by a PPRamseteCommand */
		public PathFollower(DriveBase db, String ppfile, DriverStation.Alliance alliance_transform)
			{ this(db, loadFileConstrainedTrajectory(ppfile), alliance_transform); }
		/** Follow a path planner trajectory, executed by a PPRamseteCommand */
		public PathFollower(DriveBase db, PathPlannerTrajectory t, DriverStation.Alliance alliance_transform) {
			if(alliance_transform != null && alliance_transform != DriverStation.Alliance.Invalid) {
				t = PathPlannerTrajectory.transformTrajectoryForAlliance(t, alliance_transform);
			}
			this.follower = db.followPPTrajectoryBase(t, false);
			this.drivebase = db;
			this.initial = t.getInitialPose();
		}
		/** Follow a path planner trajectory with events using the target's filename -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
		public PathFollower(DriveBase db, String ppfile, Map<String, Command> e)
			{ this(db, loadFileConstrainedTrajectory(ppfile), e); }
		/** Follow a path planner trajectory with events -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
		public PathFollower(DriveBase db, PathPlannerTrajectory t, Map<String, Command> e)
			{ this(db, t, e, DEFAULT_TRAJECTORY_ALLIANCE); }
		/** Follow a path planner trajectory with events using the target's filename -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
		public PathFollower(DriveBase db, String ppfile, Map<String, Command> e, DriverStation.Alliance alliance_transform)
			{ this(db, loadFileConstrainedTrajectory(ppfile), e, alliance_transform); }
		/** Follow a path planner trajectory with events -- executed using PathPlanner's 'FollowPathWithEvents' command (PPRamseteCommand controller) */
		public PathFollower(DriveBase db, PathPlannerTrajectory t, Map<String, Command> e, DriverStation.Alliance alliance_transform) {
			if(alliance_transform != null && alliance_transform != DriverStation.Alliance.Invalid) {
				t = PathPlannerTrajectory.transformTrajectoryForAlliance(t, alliance_transform);
			}
			this.follower = db.followEventTrajectoryBase(t, e, false);
			this.drivebase = db;
			this.initial = t.getInitialPose();
		}


		/** Whether or not the command should reset the drivebase pose on initialize */
		public PathFollower runRelative(boolean relative) {
			this.relative = relative;
			return this;
		}
		/** Whether or not the command should manually stop the drivebase when finished */
		public PathFollower stopOnEnd(boolean stop) {
			this.stop_finish = stop;
			return this;
		}


		@Override
		public void initialize() {
			if(this.relative) {
				this.drivebase.resetOdometry(this.initial);
			}
			this.follower.initialize();
		}
		@Override
		public void execute() {
			this.follower.execute();
		}
		@Override
		public boolean isFinished() {
			return this.follower.isFinished();
		}
		@Override
		public void end(boolean i) {
			this.follower.end(i);
			if(this.stop_finish) {
				this.drivebase.setDriveVoltage(0.0, 0.0);
			}
		}

		@Override
		public void initSendable(SendableBuilder b) {
			follower.initSendable(b);
			b.addBooleanProperty("Relative Start", ()->this.relative, this::runRelative);
			b.addBooleanProperty("Stop On Finish", ()->this.stop_finish, this::stopOnEnd);
		}

	}



}	// end of class
