package frc.robot;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
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
		public PathConstraints getPathConstraints() {
			return new PathConstraints(this.max_velocity, this.max_acceleration);
		}
	}

	private final Gyro 
		gyro;
	private final WPI_TalonSRX
		left, left2,
		right, right2;
	public final ClosedLoopParams
		parameters;

	public final SimpleMotorFeedforward feedforward;
	private final DifferentialDriveOdometry odometry;
	private final DifferentialDriveKinematics kinematics;

	private RamseteAutoBuilder autobuilder = null;

	private Pose2d total_tracked_pose = new Pose2d();

	public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params) {
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
		this.setNeutralMode(Constants.DRIVEBASE_NEUTRAL_MODE);
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
		return "DriveBase [2023]";
	}

	public Field2d getSendableLocation() {
		Field2d map = new Field2d();
		SmartDashboard.putData("Robot Location", map);
		return map;
	}
	public void updateFieldLocation(Field2d map) {
		map.setRobotPose(this.getTotalPose());
	}





	/* COMMAND GENERATORS */

	/** Get a tankdrive command, controlled by inputs that return input percentages (ei. joystick axis outputs: -1 to 1 )
	 * @param ds a drive supplier that supplies percentages in the range [-1, 1]
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDrivePercent(TDriveSupplier ds) {
		return new TankDrivePercent(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that return voltages for each side
	 * @param ds a drive supplier that supplies voltages in the range [-12, 12]
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVoltage(TDriveSupplier ds) {
		return new TankDriveVoltage(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that provide target velocities for each side
	 * @param ds a drive supplier that supplies velocities in METERS PER SECOND
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVelocity(TDriveSupplier ds) {
		return new TankDriveVelocity(this, ds);
	}
	/** Get a tankdrive command, controlled by inputs that provide target velocities for each side -- transitions are limited by a trapazoid profile generator
	 * @param ds a drive supplier that supplies velocities in METERS PER SECOND
	 * @return A command for driving the drivebase using tankdrive
	 */
	public CommandBase tankDriveVelocityProfiled(TDriveSupplier ds) {
		return new TankDriveVelocityProfiled(this, ds);
	}

	public CommandBase followAutoBuilderPath(String ppfile) {
		return this.getAutoBuilder(Constants.AUTO_EVENTS).followPath(
			PathPlanner.loadPath(ppfile, this.parameters.getPathConstraints()));
	}
	public CommandBase followAutoBuilderPathRelative(String ppfile) {
		return runRelativeTrajectory(this,
			PathPlanner.loadPath(ppfile, this.parameters.getPathConstraints()),
			(PathPlannerTrajectory p)->{ return this.getAutoBuilder(Constants.AUTO_EVENTS).followPath(p); }
		);
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
		this.total_tracked_pose = this.getTotalPose();
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
		Pose2d current = this.getDeltaPose();
		return this.total_tracked_pose.plus(new Transform2d(current.getTranslation(), current.getRotation()));
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

	public RamseteAutoBuilder generateAutoBuilder(HashMap<String, Command> events) {
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
	public RamseteAutoBuilder getAutoBuilder(HashMap<String, Command> events) {
		if(this.autobuilder == null) {  // or if new event map
			this.autobuilder = this.generateAutoBuilder(events);
		}
		return this.autobuilder;
	}








	/* COMMANDS */

	/** TankDrive that is controlled by 2 'voltage-returning' analog suppliers */
	public static class TankDriveVoltage extends CommandBase {
		
		private final DriveBase drivebase;
		private final TDriveSupplier driver;

		public TankDriveVoltage(DriveBase db, DoubleSupplier l, DoubleSupplier r)
			{ this(db, new TankSupplier(l, r)); }
		public TankDriveVoltage(DriveBase db, TDriveSupplier ds) {
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
		public TankDrivePercent(DriveBase db, TDriveSupplier ds) { super(db, ds); }

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
		private final TDriveSupplier driver;
		private final PIDController leftfb, rightfb;

		private TDriveSupplier.CombinedOutput
			outputs = new TDriveSupplier.CombinedOutput();
		private double la, ra;

		public TankDriveVelocity(
			DriveBase db, TDriveSupplier ds
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
		private final TDriveSupplier driver;
		private final ProfiledPIDController leftfb, rightfb;

		private TDriveSupplier.CombinedOutput
			outputs = new TDriveSupplier.CombinedOutput();
		private double la, ra;

		public TankDriveVelocityProfiled(
			DriveBase db, TDriveSupplier ds
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

	public RamseteCommand generateFollowCommand(Trajectory t) {
		return new RamseteCommand(
			t,
			this::getDeltaPose,
			this.parameters.getRamseteController(),
			this.feedforward,
			this.kinematics,
			this::getWheelSpeeds,
			this.parameters.getFeedbackController(),
			this.parameters.getFeedbackController(),
			this::setDriveVoltage
		);
	}

	public static interface TrajectoryRunnerGenerator<R extends CommandBase, T extends Trajectory> {
		public R genRunner(T trajectory);
	}
	public static<R extends CommandBase, T extends Trajectory>
		SequentialCommandGroup runRelativeTrajectory(DriveBase db, T traj, TrajectoryRunnerGenerator<R, T> gen)
	{
		return new SequentialCommandGroup(
			new PrintCommand("Beginning Relative Trajectory (resetting pose)..."),
			new InstantCommand(()->db.resetOdometry(traj.getInitialPose())),
			gen.genRunner(traj)
		);
	}


	public FollowTrajectory followTrajectory(Trajectory t) {
		return new FollowTrajectory(this, t);
	}

	public static class FollowTrajectory extends CommandBase {
		private final DriveBase drivebase; 
		private final Trajectory trajectory;
		private final RamseteCommand controller;
		private final boolean stop;

		private final PPRamseteCommand pcontroller;
		private final PathPlannerTrajectory ptrajectory;
		private final String path;

        @Override public void initialize()
        {
            drivebase.resetOdometry(this.ptrajectory.getInitialPose());
			this.pcontroller.initialize();
			System.out.println("FollowTrajectory: Running...");
        }
        @Override public void execute()
        {
            this.pcontroller.execute();
        }
        @Override public void end(boolean interrupted)
        {
            this.pcontroller.end(interrupted);
			if(this.stop) {
				drivebase.setDriveVoltage(0, 0);
			}
        }
        @Override public boolean isFinished()
        {
            return this.pcontroller.isFinished();
        }


		FollowTrajectory(DriveBase db, Trajectory t)
		{
			this(db, t, true);
		}

		FollowTrajectory(DriveBase db, Path json_path)
		{
			this(db, json_path, true);
			// calciumatator
		}

		FollowTrajectory(DriveBase db, Trajectory t, boolean s)
		{
			super();
			this.trajectory = t;
			this.stop = s;
			this.drivebase = db;

			
			this.pcontroller = null;
			this.path = null;
			this.ptrajectory = null;

			this.controller = new RamseteCommand
				(
				this.trajectory, 
				this.drivebase::getDeltaPose, 
				/* recives a function, and keeps getting new info */
				this.drivebase.parameters.getRamseteController(),
				this.drivebase.feedforward, 
				this.drivebase.kinematics,  
				this.drivebase::getWheelSpeeds, 
				this.drivebase.parameters.getFeedbackController(), 
				this.drivebase.parameters.getFeedbackController(), 
				this.drivebase::setDriveVoltage
				);
		}

		FollowTrajectory(DriveBase db, Path json_path, boolean s)
		{
			super();
			this.drivebase = db;
			this.stop = s;
			this.pcontroller = null;
			this.path = null;
			this.ptrajectory = null;
			Trajectory temp;
			try {
				temp = TrajectoryUtil.fromPathweaverJson(json_path);
			} catch(Exception e) {
				System.err.println("FAILED TO READ TRAJECTORY: " + json_path.toString() + " -> " + e.getMessage());
				temp = new Trajectory(Arrays.asList(new Trajectory.State()));	// do-nothing trajectory as placeholder
			}
			this.trajectory = temp;
			this.controller = new RamseteCommand
			(
				this.trajectory,
				this.drivebase::getDeltaPose,
				this.drivebase.parameters.getRamseteController(),
				this.drivebase.feedforward,
				this.drivebase.kinematics,
				this.drivebase::getWheelSpeeds,
				this.drivebase.parameters.getFeedbackController(), 
				this.drivebase.parameters.getFeedbackController(),
				this.drivebase::setDriveVoltage
			);
		}

	////////////////////////////////////////////////////////////////////////////////

		FollowTrajectory(DriveBase db, PathPlannerTrajectory ppt)
		{
			this(db, ppt, true);
		}

		FollowTrajectory(DriveBase db, String path)
		{
			this(db, path, true);
		}

		FollowTrajectory(DriveBase db, PathPlannerTrajectory ppt, boolean s)
		{
			super();
			this.ptrajectory = ppt;
			this.stop = s;
			this.drivebase = db;
			this.path = null;
			this.controller = null;
			this.trajectory = null;

            this.pcontroller = new PPRamseteCommand(
                ptrajectory, 
                this.drivebase::getDeltaPose,
                new RamseteController(),
                this.drivebase.feedforward,
                this.drivebase.kinematics, // DifferentialDriveKinematics
                this.drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                // vvv this is supposed to be type BiConsumer<Double, Double>
                this.drivebase::setDriveVoltage, // Voltage biconsumer
                true, // Sh(ould the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this.drivebase // Requires this drive subsystem
            );

        }

        // public PPRamseteCommand rCommand(PathPlannerTrajectory path)
        // {
        //     PPRamseteCommand command = new PPRamseteCommand(
        //         path, 
        //         this.drivebase::getDeltaPose,
        //         new RamseteController(),
        //         this.drivebase.feedforward,
        //         this.drivebase.kinematics, // DifferentialDriveKinematics
        //         this.drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
        //         new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        //         new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
        //         // vvv this is supposed to be type BiConsumer<Double, Double>
        //         this.drivebase::setDriveVoltage, // Voltage biconsumer
        //         true, // Sh(ould the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //         this.drivebase // Requires this drive subsystem
        //     );

        //     return command;
        // }

        FollowTrajectory(DriveBase db, String apath, boolean s)
        {
            super();
            this.path = apath;
            this.stop = s;
            this.drivebase = db;
            this.ptrajectory = PathPlanner.loadPath(this.path, new PathConstraints(4, 3));
            this.pcontroller = new PPRamseteCommand(
                ptrajectory, 
                this.drivebase::getDeltaPose,
                new RamseteController(),
                this.drivebase.feedforward,
                this.drivebase.kinematics, // DifferentialDriveKinematics
                this.drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                // vvv this is supposed to be type BiConsumer<Double, Double>
                this.drivebase::setDriveVoltage, // Voltage biconsumer
                true, // Sh(ould the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this.drivebase // Requires this drive subsystem
            );
            this.controller = null;
            this.trajectory = null;
        }

        public static PathPlannerTrajectory createPath(String name)
        {
            PathPlannerTrajectory examplePath = PathPlanner.loadPath(name, new PathConstraints(4, 3));
            return examplePath;
        }

	}




}	// end of class
