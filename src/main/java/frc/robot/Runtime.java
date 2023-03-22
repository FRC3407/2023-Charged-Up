package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.server.PathPlannerServer;

import frc.robot.Constants.ButtonBox;
import frc.robot.DriveBase.FollowTrajectory;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.ControlSchemeManager.AutomatedTester;
import frc.robot.team3407.commandbased.EventTriggers.*;
import frc.robot.team3407.ADIS16470_3X;


public final class Runtime extends TimedRobot {

	private static Runtime runtime = new Runtime();
	public static Runtime Get() { return Runtime.runtime; }

	private final class Robot implements Sendable {
		private final ADIS16470_3X imu_3x = new ADIS16470_3X();
		private final DriveBase drivebase = new DriveBase(
			Constants.DRIVEBASE_LAYOUT,
			this.imu_3x.getGyroAxis(Constants.IMU_YAW_AXIS),
			Constants.DRIVEBASE_PARAMS
		);
		private final Manipulator manipulator = new Manipulator(
			new Manipulator.Arm(
				Constants.ARM_WINCH_CAN_ID),
			new Manipulator.Grabber(
				Constants.GRABBER_CAN_ID,
				Constants.GRABBER_WRIST_PWM_PORT)
		);

		@Override
		public void initSendable(SendableBuilder b) {}
		public void startLogging() {
			SmartDashboard.putData("Robot", this);
			SmartDashboard.putData("Robot/IMU", this.imu_3x);
			SmartDashboard.putData("Robot/Drivebase", this.drivebase);
			SmartDashboard.putData("Robot/Manipulator/Arm", this.manipulator.arm);
			SmartDashboard.putData("Robot/Manipulator/Grabber", this.manipulator.grabber);
		}
	}
	private final Robot robot = new Robot();
	private final ControlSchemeManager controls = new ControlSchemeManager();
	private final SendableChooser<Command> auto = new SendableChooser<>();


	private Runtime() {
		this.robot.imu_3x.configRateFilter(Constants.IMU_RATE_FILTER);
	}

	@Override
	public void robotInit() {
		PortForwarder.add(1180, "10.34.7.12", 80);
		PortForwarder.add(1181, "10.34.7.12", 1181);
		Vision.init();
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		PathPlannerServer.startServer(5811);
		this.robot.startLogging();

		this.controls.addScheme("Single Xbox Controls", new AutomatedTester(Xbox.Map), this::setupXbox);
		this.controls.addScheme("Dual Xbox Controls", new AutomatedTester(Xbox.Map, Xbox.Map), this::setupXbox);
		this.controls.addScheme("Arcade Board Controls", new AutomatedTester(Attack3.Map, Attack3.Map), this::setupControlBoard);
		this.controls.addScheme("Control Board Controls", new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map), this::setupControlBoard);
		this.controls.addScheme("Competition Controls", new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map), this::setupControlBoard);
		this.controls.setAmbiguousSolution(ControlSchemeManager.AmbiguousSolution.PREFER_COMPLEX);
		this.controls.publishSelector();
		this.controls.runContinuous();

		// this.setupComp2();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS);
		FollowTrajectory follow_traj = new FollowTrajectory(this.robot.drivebase, "PathplannerLib", true);

		this.auto.setDefaultOption("Active Park (Demo)", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		this.auto.addOption("Balance Park (Demo)", Auto.balancePark(this.robot.drivebase, pitch, Constants.BALANCE_PARK_VOLTS_PER_DEGREE));
		this.auto.addOption("Follow A Trajectory", follow_traj);

		this.auto.addOption("Climb Charging Pad",
			send(Auto.climbPad(this.robot.drivebase, pitch,
				Constants.AUTO_PAD_ENGAGE_VELOCITY, Constants.AUTO_PAD_INCLINE_VELOCITY), "Commands/Climb Pad"));
		
		
				for(String t : Constants.TRAJECTORIES) {
			this.auto.addOption(
				t + " [Trajectory]",
				this.robot.drivebase.followPath(t)
			);
		}
		SmartDashboard.putData("Autonomous", this.auto);
	}
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}
	@Override
	public void disabledPeriodic() {}
	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		Command a = this.auto.getSelected();
		if(a != null) {
			AutonomousTrigger.WhileTrue(a);		// start now, end when auto ends
		} else {
			System.out.println("No auto command selected!");
		}
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





	private void setupXbox(InputDevice... inputs) {
		System.out.println("Initializing Xbox Controls!");
		InputDevice
			controller = inputs[0],
			controller2 = inputs.length > 1 ? inputs[1] : null;
		;
		TeleopTrigger.OnTrue(send(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.LY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			), "Commands/Velocity Drive")
		);
		if(controller2 != null) {	// this function can be used whether 1 or 2 are connected
			TeleopTrigger.OnTrue(send(
				new Manipulator.TestManipulator(this.robot.manipulator,
					()->Xbox.Analog.RY.getValueOf(controller2) * -1.0,
					()->Xbox.Analog.RT.getValueOf(controller2) - Xbox.Analog.LT.getValueOf(controller2),
					()->Xbox.Analog.LY.getValueOf(controller2) * 0.5 + 0.5
				), "Commands/Manipulator Test")
			);
		}
		new Vision.CameraControl(
			Xbox.Digital.A.getPressedSupplier(controller),
			Xbox.Digital.B.getPressedSupplier(controller)
		).schedule();
	}





	private void setupControlBoard(InputDevice... inputs) {
		System.out.println("Initializing Control Board Controls!");
		InputDevice
			lstick = inputs[0],
			rstick = inputs[1],
			bbox = inputs.length > 2 ? inputs[2] : null,
			controller = inputs.length > 3 ? inputs[3] : null
		;
		TeleopTrigger.OnTrue(send(
			this.robot.drivebase.tankDriveVelocity(
				Attack3.Analog.Y.getDriveInputSupplier(lstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Attack3.Analog.Y.getDriveInputSupplier(rstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			), "Commands/Velocity Drive")
		);
		if(controller != null) {
			TeleopTrigger.OnTrue(send(
				new Manipulator.TestManipulator(this.robot.manipulator,
					()->Xbox.Analog.RY.getValueOf(controller) * -1.0,		// right stick y-axis for the arm %-output
					()->Xbox.Analog.RT.getValueOf(controller) - Xbox.Analog.LT.getValueOf(controller),	// triggers for the wrist --> right+, left-
					()->Xbox.Analog.LY.getValueOf(controller) * 0.5 + 0.5		// left stick y-axis for the grabber %-output
				), "Commands/Manipulator Test")
			);
			if(bbox == null) {
				new Vision.CameraControl(
					Xbox.Digital.A.getPressedSupplier(controller),
					Xbox.Digital.B.getPressedSupplier(controller)
				).schedule();
			}
		}
		if(bbox != null) {
			new Vision.CameraControl(
				ButtonBox.Digital.B1.getPressedSupplier(bbox),
				ButtonBox.Digital.B2.getPressedSupplier(bbox)
			).schedule();
		}
	}



	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t) {
		SmartDashboard.putData(t);
		return t;
	}
	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t, String key) {
		SmartDashboard.putData(key, t);
		return t;
	}


}