package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.robot.Constants.ButtonBox;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.drive.DriveSupplier.*;
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
			Constants.DRIVEBASE_PARAMS,
			Constants.DRIVEBASE_NEUTRAL_MODE
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
	// private final Field2d field_logger = new Field2d();
	private final ControlSchemeManager controls = new ControlSchemeManager();
	private final SendableChooser<Command> auto = new SendableChooser<>();
	private BooleanSupplier auto_enable = ()->false, auto_select = ()->false;
	private CommandBase auto_driveforward, auto_balance;


	private Runtime() {
		this.robot.imu_3x.configRateFilter(Constants.IMU_RATE_FILTER);
	}

	@Override
	public void robotInit() {

		System.out.println("Using Wpilib Version " + WPILibVersion.Version);
		CommandScheduler.getInstance().registerSubsystem(this.robot.drivebase);
		Vision.init();
		if(isReal()) { DataLogManager.start(); } else { DataLogManager.start("logs/sim"); }
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
		this.robot.startLogging();

		// (new Vision.PoseUpdater(this.field_logger.getRobotObject())).schedule();
		// SmartDashboard.putData("Robot/FieldPosition", this.field_logger);

		this.controls.addScheme("Single Xbox Testing",			new AutomatedTester(Xbox.Map),											this::setupXbox,			CommandScheduler.getInstance()::cancelAll);
		this.controls.addScheme("Dual Xbox Testing",			new AutomatedTester(Xbox.Map, Xbox.Map),								this::setupXbox,			CommandScheduler.getInstance()::cancelAll);
		this.controls.addScheme("Arcade Board Controls",		new AutomatedTester(Attack3.Map, Attack3.Map),							this::setupControlBoardTD,	CommandScheduler.getInstance()::cancelAll);
		this.controls.addScheme("Control Board Controls",		new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map),			this::setupControlBoardTD,	CommandScheduler.getInstance()::cancelAll);
		this.controls.addScheme("Competition Controls (TD)",	new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map),	this::setupControlBoardTD,	CommandScheduler.getInstance()::cancelAll);
		this.controls.setDefault("Competition Controls (AD)",	new AutomatedTester(Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map),	this::setupControlBoardAD,	CommandScheduler.getInstance()::cancelAll);
		this.controls.setAmbiguousSolution(ControlSchemeManager.AmbiguousSolution.PREFER_COMPLEX);
		this.controls.publishSelector();
		this.controls.runContinuous();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(Constants.IMU_PITCH_AXIS);

		// this.auto.addOption("Active Park (Demo)", Auto.activePark(this.robot.drivebase, Constants.ACTIVE_PARK_VOLTS_PER_METER));
		// this.auto.addOption("Balance Park (Demo)", Auto.balancePark(this.robot.drivebase, pitch, Constants.BALANCE_PARK_VOLTS_PER_DEGREE));
		// this.auto.setDefaultOption("Climb Charging Pad",
		// 	send(Auto.climbPad(this.robot.drivebase, pitch,
		// 		Constants.AUTO_PAD_ENGAGE_VELOCITY, Constants.AUTO_PAD_INCLINE_VELOCITY), "Commands/Climb Pad"));
		this.auto.setDefaultOption("No Auto", null);
		for(String t : Constants.TRAJECTORIES) {
			String n = t + " [Trajectory]";
			this.auto.addOption(n, send(
				this.robot.drivebase.followEventTrajectory(t, Constants.AUTO_EVENTS, DriverStation.getAlliance()),
				"Commands/Auto Trajectories/" + n
			));
		}
		SmartDashboard.putData("Autonomous Selector", this.auto);

		this.auto_driveforward = new SequentialCommandGroup(
			Auto.driveStraight(this.robot.drivebase, -0.3, -0.8),
			new WaitCommand(0.3),
			Auto.driveStraight(this.robot.drivebase, 0.5, 0.6),
			new WaitCommand(0.5),
			Auto.driveStraight(this.robot.drivebase, -4.0, -1.2)
		);
		this.auto_balance = send(
			Auto.climbPad(
				this.robot.drivebase, pitch,
				Constants.AUTO_PAD_ENGAGE_VELOCITY,
				Constants.AUTO_PAD_INCLINE_VELOCITY
			), "Commands/Climb Charging Pad"
		);

		this.robot.manipulator.grabber.setWristAngle(Manipulator.Grabber.WRIST_MAX_ANGLE);

	}
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		Vision.PoseEstimation.fuseVision(this.robot.drivebase, true);
	}

	@Override
	public void disabledInit() {}
	@Override
	public void disabledPeriodic() {}
	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		if(this.auto_enable.getAsBoolean()) {
			System.out.println("Running Physically Selected Auto...");
			if(this.auto_select.getAsBoolean() && this.auto_balance != null) {
				this.auto_balance.schedule();
				System.out.println("Balance Auto Started!");
			} else if(this.auto_driveforward != null) {
				this.auto_driveforward.schedule();
				System.out.println("Drive Forward Auto Started!");
			}
		} else {
			System.out.println("Running NT Selected Auto...");
			Command a = this.auto.getSelected();
			if(a != null) {
				a.schedule();
			} else {
				System.out.println("No auto command selected!");
			}
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
			this.robot.drivebase.tankDriveVelocityProfiled(
				// tankDriveSupreme(
				// 	Xbox.Analog.LY.getDriveInputSupplier(controller,
				// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				// 	Xbox.Analog.RY.getDriveInputSupplier(controller,
				// 		Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
				// 	()->{ return (Xbox.Digital.RB.getValueOf(controller) && Xbox.Digital.LB.getValueOf(controller)); },
				// 	()->{ return false; },
				// 	Constants.DRIVE_ROT_RATE_SCALE, Constants.DRIVE_BOOST_SCALE, Constants.DRIVE_FINE_SCALE
				// )
				arcadeDriveSupreme(
					Xbox.Analog.RY.getDriveInputSupplier(controller,
						Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					Xbox.Analog.LX.getDriveInputSupplier(controller,
						Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE * Constants.DRIVE_ROT_RATE_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					Xbox.Digital.RB.getSupplier(controller),
					Xbox.Digital.LB.getSupplier(controller),
					Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_BOOST_SCALE, Constants.DRIVE_FINE_SCALE
				)
			), "Commands/Velocity Drive")
		);
		if(controller2 != null) {	// this function can be used whether 1 or 2 are connected
			TeleopTrigger.OnTrue(send(
				this.robot.manipulator.controlManipulatorAdv(
					Xbox.Analog.RY.getDriveInputSupplier(controller2,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),
					()->Xbox.Analog.RT.getValueOf(controller2) - Xbox.Analog.LT.getValueOf(controller2),
					Xbox.Analog.LY.getDriveInputSupplier(controller2,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),
					Xbox.Digital.LS.getPressedSupplier(controller2),
					Xbox.Digital.RB.getPressedSupplier(controller2),
					Xbox.Digital.LB.getPressedSupplier(controller2)
				), "Commands/Manipulator Control")
			);
		}
		new Vision.CameraControl(
			Xbox.Digital.A.getPressedSupplier(controller),
			Xbox.Digital.B.getPressedSupplier(controller)
		).schedule();
	}





	private void setupControlBoardTD(InputDevice... inputs) { setupControlBoard(true, inputs); }
	private void setupControlBoardAD(InputDevice... inputs) { setupControlBoard(false, inputs); }
	private void setupControlBoard(boolean tdrive, InputDevice... inputs) {
		System.out.println("Initializing Control Board Controls!");
		InputDevice
			lstick = inputs[0],
			rstick = inputs[1],
			bbox = inputs.length > 2 ? inputs[2] : null,
			controller = inputs.length > 3 ? inputs[3] : null
		;
		TeleopTrigger.OnTrue(send(
			this.robot.drivebase.tankDriveVelocityProfiled(
				tdrive ? tankDriveSupreme(
					Attack3.Analog.Y.getDriveInputSupplier(lstick,
						Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					Attack3.Analog.Y.getDriveInputSupplier(rstick,
						Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					()->{ return (Attack3.Digital.TRI.getValueOf(lstick) && Attack3.Digital.TRI.getValueOf(rstick)); },
					()->{ return (Attack3.Digital.TB.getValueOf(lstick) && Attack3.Digital.TB.getValueOf(rstick)); },
					Constants.DRIVE_ROT_RATE_SCALE, Constants.DRIVE_BOOST_SCALE, Constants.DRIVE_FINE_SCALE
				) :
				arcadeDriveSupreme(
					Attack3.Analog.Y.getDriveInputSupplier(rstick,
						Constants.DRIVE_INPUT_DEADZONE, Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					Attack3.Analog.X.getDriveInputSupplier(lstick,
						Constants.DRIVE_INPUT_DEADZONE, -1.0 * Constants.DRIVE_INPUT_VEL_SCALE * Constants.DRIVE_ROT_RATE_SCALE, Constants.DRIVE_INPUT_EXP_POWER),
					Attack3.Digital.TRI.getSupplier(rstick),
					Attack3.Digital.TRI.getSupplier(lstick),
					Constants.DRIVE_INPUT_VEL_SCALE, Constants.DRIVE_BOOST_SCALE, Constants.DRIVE_FINE_SCALE
				)
			), "Commands/Velocity Drive")
		);
		if(controller != null) {
			TeleopTrigger.OnTrue(send(
				this.robot.manipulator.controlManipulatorAdv(
					Xbox.Analog.RY.getDriveInputSupplier(controller,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),		// right stick y-axis for the arm %-output
					()->Xbox.Analog.RT.getValueOf(controller) - Xbox.Analog.LT.getValueOf(controller),	// triggers for the wrist --> right+, left-
					Xbox.Analog.LY.getDriveInputSupplier(controller,
						Constants.DRIVE_INPUT_DEADZONE, -1.0, 1.0),	// left stick y-axis for the grabber %-rate (integrated for position)
					Xbox.Digital.LS.getPressedSupplier(controller),			// press down left stick to reset wrist
					Xbox.Digital.RB.getPressedSupplier(controller),			// RB for arm lock
					Xbox.Digital.LB.getPressedSupplier(controller)			// LB for grabber lock
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
			new Vision.CameraControl.DirectSwitching(
				ButtonBox.Digital.B1.getPressedSupplier(bbox),
				ButtonBox.Digital.B2.getPressedSupplier(bbox),
				ButtonBox.Digital.B3.getPressedSupplier(bbox),
				ButtonBox.Digital.B4.getPressedSupplier(bbox),
				ButtonBox.Digital.B5.getPressedSupplier(bbox)
			).schedule();
			this.auto_enable = ButtonBox.Digital.S1.getSupplier(bbox);
			this.auto_select = ButtonBox.Digital.S2.getSupplier(bbox);
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









	private static TankSupplierRS tankDriveSupreme(
		DoubleSupplier l, DoubleSupplier r, BooleanSupplier b, BooleanSupplier f, double rs, double boost, double fine
	) {
		return new TankSupplierRS(
			()->{
				if(b.getAsBoolean() && !f.getAsBoolean()) {
					return l.getAsDouble() * boost;
				}
				if(!b.getAsBoolean() && f.getAsBoolean()) {
					return l.getAsDouble() * fine;
				}
				return l.getAsDouble();
			},
			()->{
				if(b.getAsBoolean() && !f.getAsBoolean()) {
					return r.getAsDouble() * boost;
				}
				if(!b.getAsBoolean() && f.getAsBoolean()) {
					return r.getAsDouble() * fine;
				}
				return r.getAsDouble();
			},
			()->rs
		);
	}
	private static ArcadeSupplierLM arcadeDriveSupreme(
		DoubleSupplier f, DoubleSupplier t, BooleanSupplier b, BooleanSupplier fc, double max, double boost, double fine
	) {
		return new ArcadeSupplierLM(
			()->((b.getAsBoolean() && !fc.getAsBoolean()) ? f.getAsDouble() * boost : f.getAsDouble()),
			()->((fc.getAsBoolean() && !b.getAsBoolean()) ? t.getAsDouble() * fine : t.getAsDouble()),
			()->((b.getAsBoolean() && !fc.getAsBoolean()) ? max * boost : max)
		);
	}








	// public static class LEDTest implements Sendable {

	// 	private final static double
	// 		MIN_OUTPUT_uS = 0.005,
	// 		MAX_OUTPUT_uS = 5.005
	// 	;
	// 	private final static boolean
	// 		INVERT_RANGE = false
	// 	;
	// 	private final PWM r, g, b;
	// 	private double rs, gs, bs;

	// 	public LEDTest(int rp, int gp, int bp) {
	// 		this.r = new PWM(rp);
	// 		this.g = new PWM(gp);
	// 		this.b = new PWM(bp);
	// 		this.r.setBounds(MAX_OUTPUT_uS, 0, 0, 0, MIN_OUTPUT_uS);
	// 		this.g.setBounds(MAX_OUTPUT_uS, 0, 0, 0, MIN_OUTPUT_uS);
	// 		this.b.setBounds(MAX_OUTPUT_uS, 0, 0, 0, MIN_OUTPUT_uS);
	// 		this.r.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
	// 		this.g.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
	// 		this.b.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
	// 	}

	// 	public void setRGB(double r, double g, double b) {
	// 		if(INVERT_RANGE) {
	// 			this.r.setPosition(1.0 - r);
	// 			this.g.setPosition(1.0 - g);
	// 			this.b.setPosition(1.0 - b);
	// 		} else {
	// 			this.r.setPosition(r);
	// 			this.g.setPosition(g);
	// 			this.b.setPosition(b);
	// 		}
	// 		this.rs = r;
	// 		this.gs = g;
	// 		this.bs = b;
	// 	}
	// 	public void setR(double r) {
	// 		if(INVERT_RANGE) {
	// 			this.r.setPosition(1.0 - r);
	// 		} else {
	// 			this.r.setPosition(r);
	// 		}
	// 		this.rs = r;
	// 	}
	// 	public void setG(double g) {
	// 		if(INVERT_RANGE) {
	// 			this.g.setPosition(1.0 - g);
	// 		} else {
	// 			this.g.setPosition(g);
	// 		}
	// 		this.gs = g;
	// 	}
	// 	public void setB(double b) {
	// 		if(INVERT_RANGE) {
	// 			this.b.setPosition(1.0 - b);
	// 		} else {
	// 			this.b.setPosition(b);
	// 		}
	// 		this.bs = b;
	// 	}
	// 	public void stop() {
	// 		this.r.setDisabled();
	// 		this.g.setDisabled();
	// 		this.b.setDisabled();
	// 	}

	// 	@Override
	// 	public void initSendable(SendableBuilder b) {
	// 		b.addDoubleProperty("Red channel setpoint", ()->this.rs, this::setR);
	// 		b.addDoubleProperty("Green channel setpoint", ()->this.gs, this::setG);
	// 		b.addDoubleProperty("Blue channel setpoint", ()->this.bs, this::setB);
	// 	}

	// }


}