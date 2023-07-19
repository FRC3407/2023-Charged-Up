package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import javax.swing.filechooser.FileSystemView;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.*;

import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.robot.Constants.ButtonBox;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.drive.DriveSupplier.*;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.ADIS16470_3X;
import frc.robot.team3407.SenderNT;
import frc.robot.team3407.Util;


public final class Runtime extends TimedRobot {

	public final class Robot implements Sendable {

		public final PowerDistribution power = new PowerDistribution(
			Constants.PDH_CAN_ID,
			Constants.PDH_MODULE_TYPE
		);

		public final ADIS16470_3X imu_3x = new ADIS16470_3X();

		public final DriveBase drivebase = new DriveBase(
			Constants.DRIVEBASE_LAYOUT,
			this.imu_3x.getGyroAxis(Constants.IMU_YAW_AXIS),
			Constants.DRIVEBASE_PARAMS,
			Constants.DRIVEBASE_NEUTRAL_MODE
		);

		public final Manipulator2 manipulator = new Manipulator2(
			new Manipulator2.Arm(Constants.ARM_WINCH_CAN_ID),
			new Manipulator2.Wrist.ServoImpl(Constants.GRABBER_WRIST_PWM_PORT),
			// new Manipulator2.Hand.NeverestGrabber(Constants.GRABBER_CAN_ID)
			new Manipulator2.Hand.SeatMotorGrabber(Constants.GRABBER_WRIST_PWM_PORT, 0)
		);


		private Robot() {}	// only the runtime class can construct a 'robot' object

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Power/RoboRIO VIn", RobotController::getBatteryVoltage, null);
		}
		public void startLogging() {
			SmartDashboard.putData("Robot", this);
			SmartDashboard.putData("Robot/Power", this.power);
			SmartDashboard.putData("Robot/IMU", this.imu_3x);
			SmartDashboard.putData("Robot/Drivebase", this.drivebase);
			this.manipulator.startLogging("Robot/Manipulator");
		}
		public void initialize() {
			this.drivebase.register();
			this.manipulator.register();
			this.startLogging();
		}

	}





	private final Robot robot = new Robot();
	private final Performance perf_main = new Performance();
	private final ControlSchemeManager controls = new ControlSchemeManager();

	private Performance perf_sim;
	private SenderNT sim_sender;

	private static final Runtime runtime = new Runtime();
	public static Runtime Get() { return Runtime.runtime; }
	private Runtime() {
		this.robot.imu_3x.configRateFilter(Constants.IMU_RATE_FILTER);
	}




	@Override
	public void robotInit() {

		System.out.println("Using Wpilib Version " + WPILibVersion.Version);

		this.robot.initialize();
		SmartDashboard.putData("Performance", this.perf_main);
		Vision.init();

		if(isReal()) {
			DataLogManager.start();
		} else {
			File docs_dir = new File(FileSystemView.getFileSystemView().getDefaultDirectory(), "Robot Simulation Logs");
			if(docs_dir.exists()) {
				DataLogManager.start(docs_dir.getPath());
			} else {
				DataLogManager.start("logs/sim");
			}
			this.perf_sim = new Performance();
			this.sim_sender = new SenderNT("Sim++");
			this.sim_sender.putData("Performance", this.perf_sim);
			this.robot.manipulator.logSim(this.sim_sender);
			this.addPeriodic(
				()->{
					this.perf_sim.loopInit();
					this.robot.manipulator.simTick(0.002);
					this.sim_sender.updateValues();
					this.perf_sim.loopEnd();
				},
				0.002
			);
		}
		DriverStation.startDataLog(DataLogManager.getLog());
		PathPlannerServer.startServer(5811);
		PPRamseteCommand.setLoggingCallbacks(null,
			// (PathPlannerTrajectory ppt)->{},
			(Pose2d target)->{ SmartDashboard.putNumberArray("Active Trajectory/Target Pose",
				new double[]{
					target.getX(), target.getY(),
					target.getRotation().getDegrees()
				}); },
			(ChassisSpeeds setpoint)->{
				SmartDashboard.putNumber("Active Trajectory/Setpoint Vx MpS", setpoint.vxMetersPerSecond);
				SmartDashboard.putNumber("Active Trajectory/Setpoint Vy MpS", setpoint.vyMetersPerSecond);
				SmartDashboard.putNumber("Active Trajectory/Setpoint Omega RadpS", setpoint.omegaRadiansPerSecond); },
			(Translation2d tx, Rotation2d rx)->{
				SmartDashboard.putNumber("Active Trajectory/X Error", tx.getX()); 
				SmartDashboard.putNumber("Active Trajectory/Y Error", tx.getY()); 
				SmartDashboard.putNumber("Active Trajectory/Rotation Error", rx.getDegrees()); }
		);

		Controls.setupControls(this.robot, this.controls, Controls.FeatureLevel.COMPETITION);
		this.addPeriodic(
			this.controls.genLoopableRunContinuous(),
			0.5
		);

		Auto.setHardwareOptionA(new SequentialCommandGroup(
			Auto.driveStraight(this.robot.drivebase, -0.3, -0.8),
			new WaitCommand(0.3),
			Auto.driveStraight(this.robot.drivebase, 0.5, 0.6),
			new WaitCommand(0.5),
			Auto.driveStraight(this.robot.drivebase, -4.0, -1.2)
		));
		Auto.setHardwareOptionB(Util.send(
			Auto.climbPad(
				this.robot.drivebase, this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS),
				Constants.AUTO_PAD_ENGAGE_VELOCITY,
				Constants.AUTO_PAD_INCLINE_VELOCITY
			), "Commands/Climb Charging Pad"
		));
		for(String t : Constants.TRAJECTORIES) {
			String n = t + " [Trajectory]";
			Auto.addSelectableCommand(n, Util.send(
				this.robot.drivebase.followEventTrajectory(t, Constants.AUTO_EVENTS, DriverStation.getAlliance()),
				"Commands/Auto Trajectories/" + n
			));
		}
		Auto.initialize();

	}
	@Override
	public void robotPeriodic() {
		this.perf_main.loopInit();
		Controls.updateState();
		CommandScheduler.getInstance().run();
		Vision.PoseEstimation.fuseVision(this.robot.drivebase, true);
		this.perf_main.loopEnd();
	}
	@Override
	public void simulationPeriodic() {
		this.robot.manipulator.setSimForwardAcc(this.robot.drivebase.getSimForwardAcceleration());
		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(
				this.robot.drivebase.getSimCurrentDraw(),
				this.robot.manipulator.getSimCurrentDraw()
		));
	}

	@Override
	public void disabledInit() {}
	@Override
	public void disabledPeriodic() {}
	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		Auto.runSelected();
	}
	@Override
	public void autonomousPeriodic() {}
	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {}
	@Override
	public void teleopPeriodic() {}
	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {}
	@Override
	public void testPeriodic() {}
	@Override
	public void testExit() {}





	private static final class Performance implements Sendable {

		private double
			last_loop = Timer.getFPGATimestamp(),
			loop_dt = 0.0,
			proc_dt = 0.0;

		public void loopInit() {
			double now = Timer.getFPGATimestamp();
			this.loop_dt = (now - last_loop) * 1e3;
			this.last_loop = now;
		}
		public void loopEnd() {
			this.proc_dt = (Timer.getFPGATimestamp() - last_loop) * 1e3;
		}

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Loop Time (ms)", ()->loop_dt, null);
			b.addDoubleProperty("Utilized Time (ms)", ()->proc_dt, null);
		}

	}






	// public static class SimPoseDrive extends CommandBase {

	// 	public static final double
	// 		ARM_MIN_ANGLE = -8.0,
	// 		ARM_MAX_ANGLE = 102.0,
	// 		ELBOW_REL_MIN_ANGLE = -80.0,
	// 		ELBOW_REL_MAX_ANGLE = 100.0;

	// 	private final DifferentialDriveSupplier dbsupplier;
	// 	private final DoubleSupplier arm_rate, elbow_rate;

	// 	private Pose2d robot;
	// 	private Manipulator.ManipulatorPose mpose;

	// 	public SimPoseDrive(DifferentialDriveSupplier dbs, DoubleSupplier arm, DoubleSupplier elbow) {
	// 		this.dbsupplier = dbs;
	// 		this.arm_rate = arm;
	// 		this.elbow_rate = elbow;
	// 		this.robot = new Pose2d();
	// 		this.mpose = new Manipulator.ManipulatorPose(0.0, 180.0);
	// 	}

	// 	@Override
	// 	public void initialize() {}
	// 	@Override
	// 	public void execute() {
	// 		final double DT_s = 0.02;
	// 		double
	// 			lx = this.dbsupplier.leftOutput() * DT_s,
	// 			rx = this.dbsupplier.rightOutput() * DT_s,
	// 			fx = (rx + lx) / 2.0,
	// 			tx = (rx - lx) / 0.509758; // radians
	// 		robot = robot.exp(new Twist2d(fx, 0.0, tx));
	// 		this.mpose.arm_angle = Math.min(ARM_MAX_ANGLE, Math.max(ARM_MIN_ANGLE, this.mpose.arm_angle + this.arm_rate.getAsDouble() * DT_s));
	// 		this.mpose.elbow_angle = Math.min(ELBOW_REL_MAX_ANGLE, Math.max(
	// 			Math.max(ELBOW_REL_MIN_ANGLE, Manipulator2.Kinematics.HandBBox2d.handV1MinAngle(this.mpose.arm_angle)),
	// 			this.mpose.elbow_angle + this.elbow_rate.getAsDouble() * DT_s));
	// 	}
	// 	@Override
	// 	public boolean isFinished() { return false; }
	// 	@Override
	// 	public void end(boolean i) {}
	// 	@Override
	// 	public boolean runsWhenDisabled() { return true; }

	// 	@Override
	// 	public void initSendable(SendableBuilder b) {
	// 		b.addDoubleArrayProperty("Pose2d", ()->new double[]{ this.robot.getX(), this.robot.getY(), this.robot.getRotation().getDegrees() }, null);
	// 		b.addDoubleArrayProperty("Manipulator Poses", this.mpose::getV1RawPoseData, null);
	// 		// b.addDoubleProperty("Min Angle Calculation", ()->Manipulator2.Kinematics.HandBBox2d.handV1MinAngle(this.mpose.arm_angle), null);
	// 	}

	// }


}