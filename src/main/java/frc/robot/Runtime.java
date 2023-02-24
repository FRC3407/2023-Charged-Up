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
import frc.robot.team3407.ADIS16470_3X.IMUAxis;
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

		this.controls.addScheme("Xbox Controls",
			new CompatibilityTester(Xbox.Map), this::setupXbox);
		//this.controls.addScheme("Xbox Test Controls",
		//	new CompatibilityTester(Xbox.Map, Xbox.Map), this::setupTestXbox);
		this.controls.addScheme("Arcade Controls",
			new CompatibilityTester(Attack3.Map, Attack3.Map), this::setupArcade);
		this.controls.addScheme("Control Board Controls",
			new CompatibilityTester(Attack3.Map, Attack3.Map, ButtonBox.Map), this::setupControlBoard);
		this.controls.publishSelector();
		this.controls.runInitial();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS);

		this.auto.addOption("Active Park (Demo)", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		this.auto.addOption("Balance Park (Demo)", Auto.balancePark(this.robot.drivebase, pitch, Constants.BALANCE_PARK_VOLTS_PER_DEGREE));
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
		TeleopTrigger.OnTrue(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.LY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
		EnabledTrigger.OnTrue(new Vision.CameraControl(
			controller.button(Xbox.Digital.A.value, CommandScheduler.getInstance().getDefaultButtonLoop()).rising(),
			controller.button(Xbox.Digital.B.value, CommandScheduler.getInstance().getDefaultButtonLoop()).rising()
			// Xbox.Digital.A.getCallbackFrom(controller)
		));
	}
	// private void setupTestXbox(InputDevice... inputs) {
	// 	System.out.println("Initializing Xbox Test Control Scheme.");
	// 	TeleopTrigger.OnTrue(
	// 		this.robot.drivebase.tankDriveVelocity(
	// 			Xbox.Analog.LY.getDriveInputSupplier(inputs[0],
	// 				Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
	// 			Xbox.Analog.LY.getDriveInputSupplier(inputs[1],
	// 				Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
	// 		)
	// 	);
	// }





	private void setupArcade(InputDevice... inputs) {
		System.out.println("Initializing Arcade Board Control Scheme.");
		InputDevice		// aliases for less confus
			lstick = inputs[0],
			rstick = inputs[1]
		;
		TeleopTrigger.OnTrue(
			// this.robot.drivebase.tankDriveVelocity(
			// 	Attack3.Analog.Y.getDriveInputSupplier(lstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 	Attack3.Analog.Y.getDriveInputSupplier(rstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			// )
			this.robot.drivebase.tankDrivePercent(
				Attack3.Analog.Y.getDriveInputSupplier(lstick,
					Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER),
				Attack3.Analog.Y.getDriveInputSupplier(rstick,
					Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
		(new Vision.CameraControl(
			Attack3.Digital.TRI.getCallbackFrom(rstick)
		)).schedule();
	}





	private void setupControlBoard(InputDevice... inputs) {
		System.out.println("Initializing Control Board Control Scheme.");
		InputDevice		// aliases for less confus
			lstick = inputs[0],
			rstick = inputs[1],
			bbox = inputs[2]
		;
		TeleopTrigger.OnTrue(
			// this.robot.drivebase.tankDriveVelocity(
			// 	Attack3.Analog.Y.getDriveInputSupplier(lstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 	Attack3.Analog.Y.getDriveInputSupplier(rstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			// )
			this.robot.drivebase.tankDrivePercent(
				Attack3.Analog.Y.getDriveInputSupplier(lstick,
					Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER),
				Attack3.Analog.Y.getDriveInputSupplier(rstick,
					Constants.DRIVE_INPUT_DEADZONE, -1.0, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
		(new Vision.CameraControl(
			ButtonBox.Digital.B1.getCallbackFrom(bbox)
		)).schedule();
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