package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.Input.InputDevice;


public final class Controls {

	public static enum FeatureLevel {
		TESTING,
		COMPETITION
	}
	public static enum DriveMode {
		TANK,
		ARCADE
	}

	// private static final HashMap<String, Command[]> cached_commands = new HashMap<>();


	private static void singleXbox(InputDevice... inputs) {			// single xbox controller
		
	}
	private static void dualXbox(InputDevice... inputs) {			// dual xbox controllers
		
	}
	private static void arcadeBoardSimple(InputDevice... inputs) {	// simple 2-joystick arcade board

	}
	private static void arcadeBoardFull(InputDevice... inputs) {	// 2 joysticks plus the buttonbox

	}
	private static void competitionBoard(InputDevice... inputs) {	// full control board plus an extra controller

	}




}
