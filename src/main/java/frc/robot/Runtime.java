package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.server.PathPlannerServer;

import frc.robot.Constants.ButtonBox;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.ControlSchemeManager.CompatibilityTester;
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
		Vision.init();
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		PathPlannerServer.startServer(5811);
		this.robot.startLogging();

		// new Thread(()->{	// try and debug the buttonbox
		// 	try{
		// 		Thread.sleep(2000);
		// 		InputDevice.logConnections();
		// 	} catch(InterruptedException e) {
		// 		System.out.println(e.getMessage());
		// 	}
		// }).start();

		this.controls.addScheme("Xbox Controls 2",
			new CompatibilityTester(Xbox.Map, Xbox.Map), this::setupXbox);
		// this.controls.addScheme("Arcade Controls",
		// 	new CompatibilityTester(Attack3.Map, Attack3.Map), this::setupArcade);
		// this.controls.addScheme("Control Board Controls",
		// 	new CompatibilityTester(Attack3.Map, Attack3.Map, ButtonBox.Map), this::setupControlBoard);
		this.controls.addScheme("Competition Controls",	// the buttonbox is acting weird and was not being detected so i had to remove it
			new CompatibilityTester(Attack3.Map, Attack3.Map, Xbox.Map), this::setupComp);
		this.controls.publishSelector();
		this.controls.runInitial();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS);

		this.auto.addOption("Active Park (Demo)", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		this.auto.addOption("Balance Park (Demo)", Auto.balancePark(this.robot.drivebase, pitch, Constants.BALANCE_PARK_VOLTS_PER_DEGREE));
		this.auto.addOption("Test Trajectory", this.robot.drivebase.followPath(Constants.TEST_TRAJECTORY));
		this.auto.setDefaultOption("Climb Charging Pad",
			send(Auto.climbPad(this.robot.drivebase, pitch,
				Constants.AUTO_PAD_ENGAGE_VELOCITY, Constants.AUTO_PAD_INCLINE_VELOCITY), "Commands/Climb Pad"));
		AutonomousTrigger.OnTrue(new InstantCommand(()->this.auto.getSelected().schedule()));
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
	public void autonomousInit() {}
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
		System.out.println("Initializing Xbox Control Scheme.");
		InputDevice controller = inputs[0];
		InputDevice controller2 = inputs[1];
		TeleopTrigger.OnTrue(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.LY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
		TeleopTrigger.OnTrue(send(
			new Manipulator.TestManipulator(this.robot.manipulator,
				()->Xbox.Analog.RY.getValueOf(controller2) * -1.0,
				()->Xbox.Analog.RT.getValueOf(controller2) - Xbox.Analog.LT.getValueOf(controller2),
				()->Xbox.Analog.LY.getValueOf(controller2) * -0.5 + 0.5
			), "Commands/Manipulator Test")
		);
		EnabledTrigger.OnTrue(new Vision.CameraControl(
			Xbox.Digital.A.getPressedSupplier(controller),
			Xbox.Digital.B.getPressedSupplier(controller)
		));
	}





	// private void setupArcade(InputDevice... inputs) {
	// 	System.out.println("Initializing Arcade Board Control Scheme.");
	// 	InputDevice		// aliases for less confus
	// 		lstick = inputs[0],
	// 		rstick = inputs[1]
	// 	;
	// 	TeleopTrigger.OnTrue(
	// 		// this.robot.drivebase.tankDriveVelocity(
	// 		// 	Attack3.Analog.Y.getDriveInputSupplier(lstick,
	// 		// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
	// 		// 	Attack3.Analog.Y.getDriveInputSupplier(rstick,
	// 		// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
	// 		// )
	// 		this.robot.drivebase.tankDrivePercent(
	// 			Attack3.Analog.Y.getDriveInputSupplier(lstick,
	// 				Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER),
	// 			Attack3.Analog.Y.getDriveInputSupplier(rstick,
	// 				Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER)
	// 		)
	// 	);
	// 	(new Vision.CameraControl(
	// 		Attack3.Digital.TRI.getCallbackFrom(rstick)
	// 	)).schedule();
	// }





	// private void setupControlBoard(InputDevice... inputs) {
	// 	System.out.println("Initializing Control Board Control Scheme.");
	// 	InputDevice		// aliases for less confus
	// 		lstick = inputs[0],
	// 		rstick = inputs[1],
	// 		bbox = inputs[2]
	// 	;
	// 	TeleopTrigger.OnTrue(
	// 		this.robot.drivebase.tankDriveVelocity(
	// 			Attack3.Analog.Y.getDriveInputSupplier(lstick,
	// 				Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
	// 			Attack3.Analog.Y.getDriveInputSupplier(rstick,
	// 				Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
	// 		)
	// 		// this.robot.drivebase.tankDrivePercent(
	// 		// 	Attack3.Analog.Y.getDriveInputSupplier(lstick,
	// 		// 		Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER),
	// 		// 	Attack3.Analog.Y.getDriveInputSupplier(rstick,
	// 		// 		Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER)
	// 		// )
	// 	);
	// 	// (new Vision.CameraControl(
	// 	// 	ButtonBox.Digital.B1.getCallbackFrom(bbox)

	// 	// )).schedule();
	// 	EnabledTrigger.OnTrue(new Vision.CameraControl(
	// 		bbox.button(ButtonBox.Digital.B2.value, CommandScheduler.getInstance().getDefaultButtonLoop()).rising(),
	// 		bbox.button(ButtonBox.Digital.B1.value, CommandScheduler.getInstance().getDefaultButtonLoop()).rising()
	// 	));
	// }





	private void setupComp(InputDevice... inputs) {
		System.out.println("Initializing Competition Controls!");
		InputDevice		// aliases for less confus
			lstick = inputs[0],
			rstick = inputs[1],
			controller = inputs[2]
		;
		TeleopTrigger.OnTrue(
			this.robot.drivebase.tankDriveVelocity(
				Attack3.Analog.Y.getDriveInputSupplier(lstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Attack3.Analog.Y.getDriveInputSupplier(rstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
		TeleopTrigger.OnTrue(send(
			new Manipulator.TestManipulator(this.robot.manipulator,
				()->Xbox.Analog.RY.getValueOf(controller) * -1.0,		// right stick y-axis for the arm %-output
				()->Xbox.Analog.RT.getValueOf(controller) - Xbox.Analog.LT.getValueOf(controller),	// triggers for the wrist --> right+, left-
				()->Xbox.Analog.LY.getValueOf(controller) * -0.5 + 0.5		// left stick y-axis for the grabber %-output
			), "Commands/Manipulator Test")
		);
		EnabledTrigger.OnTrue(new Vision.CameraControl(
			Xbox.Digital.A.getPressedSupplier(controller),		// A button for changing camera
			Xbox.Digital.B.getPressedSupplier(controller)		// B button for toggling overlay
		));
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