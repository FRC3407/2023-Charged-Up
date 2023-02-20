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

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.server.PathPlannerServer;

import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.controls.ControlSchemeManager;
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
			this.imu_3x.getGyroAxis(IMUAxis.kZ),
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
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		PathPlannerServer.startServer(5811);
		this.robot.startLogging();

		this.controls.addScheme("Xbox Controls",
			(InputDevice... inputs)->{
				for(int i = 0; i < inputs.length; i++) {
					if(Xbox.Analog.TOTAL.compatible(inputs[i]) && Xbox.Digital.TOTAL.compatible(inputs[i])) {
						return new int[]{i};
					}
				}
				return null;
			},
			this::setupXbox
		);
		this.controls.addScheme("Arcade Controls",
			(InputDevice... inputs)->{
				int[] ret = new int[2];
				int found = 0;
				for(int i = 0; i < inputs.length; i++) {
					if(Attack3.Analog.TOTAL.compatible(inputs[i]) && Attack3.Digital.TOTAL.compatible(inputs[i])) {
						ret[found] = i;
						found++;
						if(found == 2) {
							return ret;
						}
					}
				}
				return null;
			},
			this::setupArcade
		);
		this.controls.publishSelector();
		this.controls.runInitial();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(IMUAxis.kX);

		this.auto.addOption("Active Park", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		this.auto.setDefaultOption("Climb Charging Pad",
			send(Auto.climbPad(this.robot.drivebase, pitch, Constants.AUTO_PAD_INCLINE_VELOCITY), "Commands/Climb Pad"));
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
		TeleopTrigger.OnTrue(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.LY.getDriveInputSupplier(inputs[0],
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(inputs[0],
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
	}
	private void setupArcade(InputDevice... inputs) {
		System.out.println("Initializing Arcade Board Control Scheme.");
		TeleopTrigger.OnTrue(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.RY.getDriveInputSupplier(inputs[0],
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(inputs[1],
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER)
			)
		);
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