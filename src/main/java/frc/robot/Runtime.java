package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import com.pathplanner.lib.server.PathPlannerServer;

import frc.robot.Constants.ButtonBox;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.ControlSchemeManager.AutomatedTester;
import frc.robot.team3407.commandbased.EventTriggers.*;
import frc.robot.team3407.ADIS16470_3X;


public final class Runtime extends TimedRobot {

	private static Runtime runtime = new Runtime();
	public static Runtime Get() { return Runtime.runtime; }

	private final class Robot implements Sendable {
		private final PowerDistribution power = new PowerDistribution(
			Constants.PDH_CAN_ID,
			Constants.PDH_MODULE_TYPE
		);
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
		public void initSendable(SendableBuilder b) {
			this.power.resetTotalEnergy();
			b.addDoubleProperty("Power/Temperature", this.power::getTemperature, null);
			b.addDoubleProperty("Power/Total Power (W)", this.power::getTotalPower, null);
			b.addDoubleProperty("Power/Total Energy Used (J)", this.power::getTotalEnergy, null);	// divide by 3600 for watt-hours
		}
		public void startLogging() {
			SmartDashboard.putData("Robot", this);
			SmartDashboard.putData("Robot/Power", this.power);
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
		if(isReal()) { DataLogManager.start(); }
		else { DataLogManager.start("logs/sim"); }
		DriverStation.startDataLog(DataLogManager.getLog());
		PathPlannerServer.startServer(5811);
		this.robot.startLogging();

		this.controls.addScheme("Single Xbox Controls", new AutomatedTester(Xbox.Map), this::setupXbox);
		this.controls.addScheme("Dual Xbox Controls", new AutomatedTester(Xbox.Map, Xbox.Map), this::setupXbox);
		this.controls.addScheme("Arcade Board Controls", new AutomatedTester(Attack3.Map, Attack3.Map), this::setupControlBoard);
		this.controls.addScheme("Control Board Controls", new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map), this::setupControlBoard);
		this.controls.setDefault("Competition Controls", new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map), this::setupControlBoard);
		this.controls.setAmbiguousSolution(ControlSchemeManager.AmbiguousSolution.PREFER_COMPLEX);
		this.controls.publishSelector();
		this.controls.runContinuous();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS);

		this.auto.setDefaultOption("Active Park (Demo)", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		this.auto.addOption("Balance Park (Demo)", Auto.balancePark(this.robot.drivebase, pitch, Constants.BALANCE_PARK_VOLTS_PER_DEGREE));
		this.auto.addOption("Climb Charging Pad",
			send(Auto.climbPad(this.robot.drivebase, pitch,
				Constants.AUTO_PAD_ENGAGE_VELOCITY, Constants.AUTO_PAD_INCLINE_VELOCITY), "Commands/Climb Pad"));
		for(String t : Constants.TRAJECTORIES) {
			this.auto.addOption(
				t + " [Trajectory]",
				this.robot.drivebase.followAutoBuilderPathRelative(t)
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
			// this.robot.drivebase.tankDriveVelocityProfiled(
			// 	Xbox.Analog.LY.getDriveInputSupplier(controller,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 	Xbox.Analog.RY.getDriveInputSupplier(controller,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 	Constants.DRIVE_ROT_RATE_SCALE
			// ), "Commands/Velocity Drive")
			new TankDriveSupreme(this.robot.drivebase,
				Xbox.Analog.LY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Xbox.Analog.RY.getDriveInputSupplier(controller,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				()->{ return (Xbox.Digital.RB.getValueOf(controller) && Xbox.Digital.LB.getValueOf(controller)); },
				()->{ return false; },
				Constants.DRIVE_BOOST_PERCENT,
				Constants.DRIVE_ROT_RATE_SCALE,
				Constants.DRIVE_FINE_CONTROL_RRS
			), "Commands/Velocity Drive")
		);
		if(controller2 != null) {	// this function can be used whether 1 or 2 are connected
			TeleopTrigger.OnTrue(send(
				new Manipulator.TestManipulator(this.robot.manipulator,
					()->Xbox.Analog.RY.getValueOf(controller2) * -1.0,
					()->Xbox.Analog.RT.getValueOf(controller2) - Xbox.Analog.LT.getValueOf(controller2),
					Xbox.Analog.LY.getDriveInputSupplier(controller2,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),
					Xbox.Digital.LB.getPressedSupplier(controller2)
				), "Commands/Manipulator Control")
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
			// this.robot.drivebase.tankDriveVelocityProfiled(
			// 	Attack3.Analog.Y.getDriveInputSupplier(lstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 	Attack3.Analog.Y.getDriveInputSupplier(rstick,
			// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
			// 		Constants.DRIVE_ROT_RATE_SCALE
			// ), "Commands/Velocity Drive")
			new TankDriveSupreme(this.robot.drivebase,
				Attack3.Analog.Y.getDriveInputSupplier(lstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				Attack3.Analog.Y.getDriveInputSupplier(rstick,
					Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				()->{ return (Attack3.Digital.TRI.getValueOf(lstick) && Attack3.Digital.TRI.getValueOf(rstick)); },
				()->{ return (Attack3.Digital.TB.getValueOf(lstick) && Attack3.Digital.TB.getValueOf(rstick)); },
				Constants.DRIVE_BOOST_PERCENT,
				Constants.DRIVE_ROT_RATE_SCALE,
				Constants.DRIVE_FINE_CONTROL_RRS
			), "Commands/Velocity Drive")
		);
		if(controller != null) {
			TeleopTrigger.OnTrue(send(
				new Manipulator.TestManipulator(this.robot.manipulator,
					()->Xbox.Analog.RY.getValueOf(controller) * -1.0,		// right stick y-axis for the arm %-output
					()->Xbox.Analog.RT.getValueOf(controller) - Xbox.Analog.LT.getValueOf(controller),	// triggers for the wrist --> right+, left-
					Xbox.Analog.LY.getDriveInputSupplier(controller,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),	// left stick y-axis for the grabber %-rate (integrated for position)
					Xbox.Digital.LB.getPressedSupplier(controller)			// RB on the xbox to reset wrist position
				), "Commands/Manipulator Control")
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









	private static class TankDriveSupreme extends CommandBase {

		private final DriveBase.TankDriveVelocityProfiled driver;
		private final DriveBase drivebase;
		private final DoubleSupplier
			leftv, rightv;
		private final BooleanSupplier
			boost, finecontrol;
		private final double
			bpercent, rscale, frscale;

		public TankDriveSupreme(
			DriveBase db,
			DoubleSupplier lv, DoubleSupplier rv,
			BooleanSupplier b, BooleanSupplier f,
			double bpcnt, double rs, double frs
		) {
			this.drivebase = db;
			this.driver = new DriveBase.TankDriveVelocityProfiled(db,
				this::lVelSupplier, this::rVelSupplier, this::rscaleSupplier);
			this.leftv = lv;
			this.rightv = rv;
			this.boost = b;
			this.finecontrol = f;
			this.bpercent = bpcnt;
			this.rscale = rs;
			this.frscale = frs;
			super.addRequirements(db);
		}

		private double lVelSupplier() {
			double l = this.leftv.getAsDouble();
			return this.boost.getAsBoolean() ? l * bpercent / 100.0 : l;
		}
		private double rVelSupplier() {
			double r = this.rightv.getAsDouble();
			return this.boost.getAsBoolean() ? r * bpercent / 100.0 : r;
		}
		private double rscaleSupplier() {
			return this.finecontrol.getAsBoolean() ? this.frscale : this.rscale;
		}

		@Override
		public void initialize() {
			this.driver.initialize();
		}
		@Override
		public void execute() {
			this.driver.execute();
		}
		@Override
		public boolean isFinished() {
			return this.driver.isFinished();
		}
		@Override
		public void end(boolean i) {
			this.driver.end(i);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			this.driver.initSendable(b);
		}

	}


}