package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBox;
import frc.robot.Runtime.Robot;
import frc.robot.team3407.Util;
import frc.robot.team3407.commandbased.EventTriggers.TeleopTrigger;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.ControlSchemeManager.AmbiguousSolution;
import frc.robot.team3407.controls.ControlSchemeManager.AutomatedTester;
import frc.robot.team3407.controls.ControlSchemeManager.ControlScheme;
import frc.robot.team3407.controls.ControlSchemeManager.ControlSchemeBase;
import frc.robot.team3407.controls.Input.AnalogMap;
import frc.robot.team3407.controls.Input.AnalogSupplier;
import frc.robot.team3407.controls.Input.Attack3;
import frc.robot.team3407.controls.Input.DigitalMap;
import frc.robot.team3407.controls.Input.InputDevice;
import frc.robot.team3407.controls.Input.InputMap;
import frc.robot.team3407.controls.Input.Xbox;
import frc.robot.team3407.drive.DriveSupplier.ArcadeSupplierLM;
import frc.robot.team3407.drive.DriveSupplier.DifferentialDriveSupplier;
import frc.robot.team3407.drive.DriveSupplier.TankSupplierRS;


public final class Controls {

	public static enum FeatureLevel {
		TESTING,
		COMPETITION
	}
	public static enum DriveMode {
		TANK,
		ARCADE
	}

	public static final FeatureLevel DEFAULT_FEATURE_LEVEL = FeatureLevel.TESTING;
	public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.TANK;

	private static final EventLoop loop = new EventLoop();
	private static final Trigger
		// enabled_trigger = EnabledTrigger.makeWithLoop(loop),		// uncomment if these actually ever get used
		// disabled_trigger = DisabledTrigger.makeWithLoop(loop),
		teleop_trigger = TeleopTrigger.makeWithLoop(loop);
	private static Command[] active_commands;

	private static ControlScheme[] getAllSchemes(Robot robot) {
		return new ControlScheme[]{
			buildScheme("Single Xbox",		(InputDevice... i)->singleXbox(robot, DEFAULT_DRIVE_MODE, i),			Xbox.Map),
			buildScheme("Dual Xbox",			(InputDevice... i)->dualXbox(robot, DEFAULT_DRIVE_MODE, i),				Xbox.Map, Xbox.Map),
			buildScheme("Simple Arcade",		(InputDevice... i)->arcadeBoardSimple(robot, DEFAULT_DRIVE_MODE, i),	Attack3.Map, Attack3.Map),
			buildScheme("Full Arcade",		(InputDevice... i)->arcadeBoardFull(robot, DEFAULT_DRIVE_MODE, i),		Attack3.Map, Attack3.Map, ButtonBox.Map),
			buildScheme("Competition Board",	(InputDevice... i)->competitionBoard(robot, DEFAULT_DRIVE_MODE, i),		Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map)
		};
	}
	private static ControlScheme[] getCompetitionSchemes(Robot robot) {
		ControlScheme
			tank = buildScheme("Competition Controls (Tank Drive)",		(InputDevice... i)->competitionBoard(robot, DriveMode.TANK, i),		Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map),
			arcade = buildScheme("Competition Controls (Arcade Drive)",	(InputDevice... i)->competitionBoard(robot, DriveMode.ARCADE, i),	Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map);
		return DEFAULT_DRIVE_MODE == DriveMode.TANK ?
			new ControlScheme[]{tank, arcade} : new ControlScheme[]{arcade, tank};
	}


	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager) { return setupControls(robot, manager, null); }
	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager, FeatureLevel features) {
		if(manager == null) { manager = new ControlSchemeManager(); }
		if(features == null) { features = DEFAULT_FEATURE_LEVEL; }
		switch(features) {
			case TESTING: {
				ControlScheme[] schemes = getAllSchemes(robot);
				for(ControlScheme sch : schemes) {
					manager.addScheme(sch);
				}
				break;
			}
			case COMPETITION: {
				ControlScheme[] schemes = getCompetitionSchemes(robot);
				manager.setDefault(schemes[0]);
				for(int i = 1; i < schemes.length; i++) {
					manager.addScheme(schemes[i]);
				}
				break;
			}
		}
		manager.setAmbiguousSolution(AmbiguousSolution.PREFER_COMPLEX);
		manager.publishSelector("Control Scheme");
		return manager;
	}
	public static void updateState() {
		loop.poll();
	}

	private static ControlScheme buildScheme(String name, ControlSchemeBase.Setup_F setup, InputMap... reqs) {
		return new ControlScheme(name, new AutomatedTester(reqs), setup, Controls::deScheduleActive);
	}
	private static void deScheduleActive() {
		for(Command c : active_commands) {
			c.cancel();
		}
		loop.clear();
		Auto.clearHardwareSelectors();
	}





	private static DifferentialDriveSupplier tankDriveSupreme(
		DoubleSupplier left_rate, DoubleSupplier right_rate,
		BooleanSupplier boost_state, BooleanSupplier limited_state,
		double rotation_scale, double boost_scale, double limited_scale
	) {
		return new TankSupplierRS(
			()->{
				if(boost_state.getAsBoolean() && !limited_state.getAsBoolean()) {
					return left_rate.getAsDouble() * boost_scale;
				}
				if(!boost_state.getAsBoolean() && limited_state.getAsBoolean()) {
					return left_rate.getAsDouble() * limited_scale;
				}
				return left_rate.getAsDouble();
			},
			()->{
				if(boost_state.getAsBoolean() && !limited_state.getAsBoolean()) {
					return right_rate.getAsDouble() * boost_scale;
				}
				if(!boost_state.getAsBoolean() && limited_state.getAsBoolean()) {
					return right_rate.getAsDouble() * limited_scale;
				}
				return right_rate.getAsDouble();
			},
			()->rotation_scale
		);
	}
	private static DifferentialDriveSupplier arcadeDriveSupreme(
		DoubleSupplier fwd_rate, DoubleSupplier turn_rate,
		BooleanSupplier boost_state, BooleanSupplier limited_state,
		double max_rate, double boost_scale, double limited_scale
	) {
		return new ArcadeSupplierLM(
			()->((boost_state.getAsBoolean() && !limited_state.getAsBoolean()) ?
				fwd_rate.getAsDouble() * boost_scale : fwd_rate.getAsDouble()),
			()->(((limited_state.getAsBoolean() && !boost_state.getAsBoolean()) ?
				turn_rate.getAsDouble() * limited_scale : turn_rate.getAsDouble()) /* (fwd_rate.getAsDouble() < 0 ? -1 : 1)*/),
			()->((boost_state.getAsBoolean() && !limited_state.getAsBoolean()) ?		// ^^ invert turn if going backwards because otherwise it will be opposite that of the stick direction
				max_rate * boost_scale : max_rate)
		);
	}





	private static final double
		DEADZONE = Constants.DRIVE_INPUT_DEADZONE,
		EXP_MODIFIER = Constants.DRIVE_INPUT_EXP_POWER,
		MAX_DRIVE_VELOCITY = Constants.DRIVE_INPUT_VEL_SCALE,
		ROTATION_RATE_SCALE = Constants.DRIVE_ROT_RATE_SCALE,
		DRIVE_BOOST = Constants.DRIVE_BOOST_SCALE,
		DRIVE_LIMITED = Constants.DRIVE_FINE_SCALE;


	private static DifferentialDriveSupplier standardTankDriveSupreme(
		DoubleSupplier left_rate, DoubleSupplier right_rate,
		BooleanSupplier boost_state, BooleanSupplier limited_state
	) {
		return tankDriveSupreme(left_rate, right_rate, boost_state, limited_state, ROTATION_RATE_SCALE, DRIVE_BOOST, DRIVE_LIMITED);
	}
	private static DifferentialDriveSupplier standardArcadeDriveSupreme(
		DoubleSupplier fwd_rate, DoubleSupplier turn_rate,
		BooleanSupplier boost_state, BooleanSupplier limited_state
	) {
		return arcadeDriveSupreme(fwd_rate, turn_rate, boost_state, limited_state, MAX_DRIVE_VELOCITY, DRIVE_BOOST, DRIVE_LIMITED);
	}

	private static AnalogSupplier standardDriveSupplier(AnalogMap x, InputDevice i, double scale) {
		return x.getDriveInputSupplier(i, DEADZONE, scale, EXP_MODIFIER);
	}
	private static AnalogSupplier XMinusY(AnalogMap x, AnalogMap y, InputDevice i) {
		return ()->x.getValueOf(i) - y.getValueOf(i);
	}
	private static AnalogSupplier XMinusYScaled(AnalogMap x, AnalogMap y, InputDevice i, double scale) {
		return ()->((x.getValueOf(i) - y.getValueOf(i)) * scale);
	}
	private static BooleanSupplier ANotB(DigitalMap a, DigitalMap b, InputDevice i) {
		return ANotB(a, b, i, i);
	}
	private static BooleanSupplier ANotB(DigitalMap a, DigitalMap b, InputDevice ai, InputDevice bi) {
		return ()->a.getValueOf(ai) && !b.getValueOf(bi);
	}


	private static void setupBaseTeleopControls(CommandBase dbcontrol, CommandBase mcontrol) {
		teleop_trigger
			.onTrue(Util.send(dbcontrol, "Commands/Velocity Drive"))
			.onTrue(Util.send(mcontrol, "Commands/Manipulator Control"));
	}





	private static void singleXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {			// single xbox controller
		InputDevice controller = inputs[0];

		DifferentialDriveSupplier dds;
		// switch(drivemode) {
		// 	default:
		// 	case TANK: {
		// 		dds = standardTankDriveSupreme(
		// 			standardDriveSupplier(Xbox.Analog.LY, controller, MAX_DRIVE_VELOCITY),
		// 			standardDriveSupplier(Xbox.Analog.RY, controller, MAX_DRIVE_VELOCITY),
		// 			ANotB(Xbox.Digital.RB, Xbox.Digital.LB, controller),
		// 			ANotB(Xbox.Digital.LB, Xbox.Digital.RB, controller)
		// 		);
		// 		break;
		// 	}
		// 	case ARCADE: {
		// 		dds = standardArcadeDriveSupreme(
		// 			standardDriveSupplier(Xbox.Analog.RY, controller, MAX_DRIVE_VELOCITY),
		// 			standardDriveSupplier(Xbox.Analog.LX, controller, MAX_DRIVE_VELOCITY * -1.0 * ROTATION_RATE_SCALE),
		// 			ANotB(Xbox.Digital.RB, Xbox.Digital.LB, controller),
		// 			ANotB(Xbox.Digital.LB, Xbox.Digital.RB, controller)
		// 		);
		// 	}
		// }
		dds = standardArcadeDriveSupreme(
			XMinusYScaled(Xbox.Analog.RT, Xbox.Analog.LT, controller, -MAX_DRIVE_VELOCITY),
			standardDriveSupplier(Xbox.Analog.LX, controller, MAX_DRIVE_VELOCITY * -1.0 * ROTATION_RATE_SCALE),
			ANotB(Xbox.Digital.RB, Xbox.Digital.LB, controller),
			ANotB(Xbox.Digital.LB, Xbox.Digital.RB, controller)
		);
		CommandBase
			drive_control = robot.drivebase.tankDriveVelocityProfiled(dds),
			mpl_control = robot.manipulator.controlManipulatorAdv(
				Xbox.Analog.RY.getDriveInputSupplier(controller, DEADZONE, -1.0, 1.0),		// arm output
				Xbox.Analog.RT.getSupplier(controller), // wrist up
				Xbox.Analog.LT.getSupplier(controller), // wrist down
				Xbox.Digital.A.getPressedSupplier(controller),		// arm lock
				()->false, // grab lock
				Xbox.Digital.RB.getSupplier(controller),    //wheel intake right
				Xbox.Digital.LB.getSupplier(controller)    //wheel intake left
			);

		setupBaseTeleopControls(drive_control, mpl_control);

		active_commands = new Command[] {
			drive_control,
			mpl_control
		};

	}





	private static void dualXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {			// dual xbox controllers
		InputDevice controller1 = inputs[0];
		InputDevice controller2 = inputs[1];

		DifferentialDriveSupplier dds;
		switch(drivemode) {
			default:
			case TANK: {
				dds = standardTankDriveSupreme(
					standardDriveSupplier(Xbox.Analog.LY, controller1, MAX_DRIVE_VELOCITY),
					standardDriveSupplier(Xbox.Analog.RY, controller1, MAX_DRIVE_VELOCITY),
					ANotB(Xbox.Digital.RB, Xbox.Digital.LB, controller1),
					ANotB(Xbox.Digital.LB, Xbox.Digital.RB, controller1)
				);
				break;
			}
			case ARCADE: {
				dds = standardArcadeDriveSupreme(
					standardDriveSupplier(Xbox.Analog.RY, controller1, MAX_DRIVE_VELOCITY),
					standardDriveSupplier(Xbox.Analog.LX, controller1, MAX_DRIVE_VELOCITY * -1.0 * ROTATION_RATE_SCALE),
					ANotB(Xbox.Digital.RB, Xbox.Digital.LB, controller1),
					ANotB(Xbox.Digital.LB, Xbox.Digital.RB, controller1)
				);
			}
		}
		CommandBase
			drive_control = robot.drivebase.tankDriveVelocityProfiled(dds),
			mpl_control = robot.manipulator.controlManipulatorAdv(
				Xbox.Analog.RY.getDriveInputSupplier(controller2, DEADZONE, -1.0, 1.0),
				Xbox.Analog.RT.getSupplier(controller2), // wrist up
				Xbox.Analog.LT.getSupplier(controller2), // wrist down
				// Xbox.Analog.LY.getDriveInputSupplier(controller2, DEADZONE, -1.0, 1.0),
				// Xbox.Digital.LS.getPressedSupplier(controller2),
				// Xbox.Digital.RB.getPressedSupplier(controller2),
				Xbox.Digital.A.getPressedSupplier(controller2),		// arm lock
				()->false, // grab lock
				Xbox.Digital.RB.getSupplier(controller2),    //wheel intake right
				Xbox.Digital.LB.getSupplier(controller2)     //wheel intake left
			);

		setupBaseTeleopControls(drive_control, mpl_control);

		active_commands = new Command[] {
			drive_control,
			mpl_control
		};

	}





	private static void arcadeBoardSimple(Robot robot, DriveMode drivemode, InputDevice... inputs) {	// simple 2-joystick arcade board
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];

		

	}





	private static void arcadeBoardFull(Robot robot, DriveMode drivemode, InputDevice... inputs) {	// 2 joysticks plus the buttonbox
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];

		

	}





	private static void competitionBoard(Robot robot, DriveMode drivemode, InputDevice... inputs) {	// full control board plus an extra controller
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		InputDevice controller = inputs[3];

		DifferentialDriveSupplier dds;
		switch(drivemode) {
			default:
			case TANK: {
				dds = standardTankDriveSupreme(
					standardDriveSupplier(Attack3.Analog.Y, lstick, MAX_DRIVE_VELOCITY),
					standardDriveSupplier(Attack3.Analog.Y, rstick, MAX_DRIVE_VELOCITY),
					ANotB(Attack3.Digital.TRI, Attack3.Digital.TRI, rstick, lstick),
					ANotB(Attack3.Digital.TRI, Attack3.Digital.TRI, lstick, rstick)
				);
				break;
			}
			case ARCADE: {
				dds = standardArcadeDriveSupreme(
					standardDriveSupplier(Attack3.Analog.Y, rstick, MAX_DRIVE_VELOCITY),
					standardDriveSupplier(Attack3.Analog.X, lstick, MAX_DRIVE_VELOCITY * -1.0 * ROTATION_RATE_SCALE),
					ANotB(Attack3.Digital.TRI, Attack3.Digital.TRI, rstick, lstick),
					ANotB(Attack3.Digital.TRI, Attack3.Digital.TRI, lstick, rstick)
				);
			}
		}
		CommandBase
			drive_control = robot.drivebase.tankDriveVelocityProfiled(dds),
			mpl_control = robot.manipulator.controlManipulatorAdv(
				Xbox.Analog.RY.getDriveInputSupplier(controller, DEADZONE, -1.0, 1.0),		// arm output
				Xbox.Analog.RT.getSupplier(controller), // wrist up
				Xbox.Analog.LT.getSupplier(controller), // wrist down
				Xbox.Digital.A.getPressedSupplier(controller),		// arm lock
				()->false, // grab lock
				Xbox.Digital.RB.getSupplier(controller),    //wheel intake right
				Xbox.Digital.LB.getSupplier(controller)    //wheel intake left
			);

		setupBaseTeleopControls(drive_control, mpl_control);

		Auto.setHardwareSelectors(
			ButtonBox.Digital.S1.getSupplier(bbox),
			ButtonBox.Digital.S2.getSupplier(bbox)
		);

		// schedule camera control via buttonbox

		active_commands = new Command[] {
			drive_control,
			mpl_control
		};

	}


}
