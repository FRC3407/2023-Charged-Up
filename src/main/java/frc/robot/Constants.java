package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.team3407.drive.*;
import frc.robot.team3407.drive.Types.*;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.ADIS16470_3X.IMUAxis;


public final class Constants {

	public static final DriveMap_4<WPI_TalonSRX>
		DRIVEBASE_LAYOUT = new DriveMap_4<>(
			5, 2, 4, 3,			// note that the 'front' motors for each side should be the ones that have the encoders plugged in
			Motors.can_talonsrx,
			Inversions.RIGHT,
			DriveLayout.DIFFERENTIAL
		);
	public static final NeutralMode
		DRIVEBASE_NEUTRAL_MODE = NeutralMode.Brake;
	public static final DriveBase.ClosedLoopParams
		DRIVEBASE_PARAMS = new DriveBase.ClosedLoopParams(  // MAKE SURE THESE ARE ALL CORRECT BEFORE TEST DRIVING IT!!!
			0.58204,		// use the 'empirical' value from characterization here rather than the actual width -- actual is 0.51615848
			0.1524,		// 6 inches, coverted to meters
			0.77652,		// angular is 1.239
			2.1418,			// angular is 2.3416
			0.81806,		// angular is 0.50996
			2.8089,			// angular is 3.0669
			0.0,
			0.0,
			2.0,
			0.7,
			10.0,
			2.5,
			50.0,
			50.0,
			Inversions.BOTH
		);
	public static final int
		PDH_CAN_ID =				1,
		ARM_WINCH_CAN_ID =			10,
		GRABBER_CAN_ID =			11,
		ARM_EXTENDER_CAN_ID =		12,

		GRABBER_WRIST_PWM_PORT =	0
	;

	public static final ModuleType
		PDH_MODULE_TYPE = ModuleType.kRev;
	public static final IMUAxis
		IMU_YAW_AXIS = IMUAxis.kZ,
		IMU_PITCH_AXIS = IMUAxis.kY		// on the new robot the IMU is turned sidewasy so pitch is kY, not kX like before
	;

	public static final double
		DRIVE_INPUT_DEADZONE = 0.05,
		DRIVE_INPUT_VEL_SCALE = -DRIVEBASE_PARAMS.max_velocity,
		DRIVE_INPUT_EXP_POWER = 1.0,
		DRIVE_ROT_RATE_SCALE = 0.5,
		DRIVE_BOOST_SCALE = 1.5,
		DRIVE_FINE_SCALE = 0.5,

		TRAJECTORY_MAX_VEL = 1.5,
		TRAJECTORY_MAX_ACC = 2.0,

		IMU_RATE_FILTER = 0.40,

		ACTIVE_PARK_VOLTS_PER_METER = 100.0,
		BALANCE_PARK_VOLTS_PER_DEGREE = 0.2,
		AUTO_PAD_ENGAGE_VELOCITY = 1.0,
		AUTO_PAD_INCLINE_VELOCITY = 0.5,	// set to 0.1 if we ever fix ff/fb for inclines

		ARM_ANGLE_KF = 1.0,
		ARM_ANGLE_KP = 1.0,
		ARM_ANGLE_KI = 0.0,
		ARM_ANGLE_KD = 0.0,
		ARM_ANGLE_CRUISE_DEG_PER_SEC = 60.0,	// mike said ~10-15% output for cruising speed
		ARM_ANGLE_ACC_DEG_PER_SEC_SQRD = 120.0,

		GRAB_POSITION_KF = 1.0,
		GRAB_POSITION_KP = 1.0,
		GRAB_POSITION_KI = 0.0,
		GRAB_POSITION_KD = 0.0
	;

	public static final double
		GRABBER_ROT_RADIUS_INCHES = 4.9787,				// the radius of the gear-driven linkage
		GRABBER_FINGER_OFFSET_INCHES = 1.5093,			// offset between linkage pivot and finger "grab surface"
		GRABBER_PIVOT_OFFSET_INCHES = 1.8889,			// offset between large gear pivot and "center" - half the distance between large gear centers
		GRABBER_GEARING_IN2OUT = (28.0 / 12.0),			// Input has 12 teeth, output has 28 teeth - "input rotations per output rotations"
		GRABBER_A0_OFFSET = 120,						// difference in angle from when the grabber is at it's max angle compared to when the fingers are touching
		GRABBER_A0_WIDTH_INCHES = Manipulator.Grabber.grabAngleToWidth(0.0),
		GRABBER_W0_ANGLE_DEGREES = Manipulator.Grabber.grabWidthToAngle(0.0)
	;

	public static final int		// see: https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
		SRX_MAG_UNITS_PER_REVOLUTION = 4096,
		FALCON_UNITS_PER_REVOLUTION = 2048,
		ANALOG_UNITS_PER_REVOLUTION = 1024,		// or for whatever range the analog device has
		NEVEREST_UNITS_PER_REVOLUTION = (7 * 60 * 4),	// 7 pulse/rot * 60:1 gearing * 4 counts/pulse as counted by TalonSRX
		SEAT_MOTOR_COUNTS_PER_REVOLUTION = 175
	;

	public static final String
		TEST_AUTO = "Test Auto",
		TEST_FOLLOW = "Test Follow";
	public static final String[]		// make a list so we can automate adding all the trajectories as selectable auto options
		TRAJECTORIES = new String[]{
			TEST_AUTO, TEST_FOLLOW, "T1", "T2", "T3", "Lines"
		};
	public static final HashMap<String, Command> AUTO_EVENTS = new HashMap<>();



	public static class ButtonBox extends InputMap {
		public static enum Digital implements DigitalMap {
			B1(1), B2(2), B3(3), B4(4), B5(5), B6(6),
			S1(7), S2(8),
			TOTAL(16);	// whatever interface board the bbox is using apparently has 12 buttons and 1 POV

			public final int value;
			private Digital(int v) { this.value = v; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private ButtonBox() {}
		public static final ButtonBox Map = new ButtonBox();
		public static final int AXIS_COUNT = 5;		// see the last comment --> the bbox apparently has 5 axis

		public boolean compatible(GenericHID i)
			{ return Digital.TOTAL.compatible(i) && i.getAxisCount() == AXIS_COUNT; }
		public boolean compatible(int p)
			{ return Digital.TOTAL.compatible(p) && DriverStation.getStickAxisCount(p) == AXIS_COUNT; }
	}






	// sourced from https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/FieldConstants.java
	public static final class Field {

		public static final double
			FIELD_LENGTH = Units.inchesToMeters(651.25),
			FIELD_WIDTH = Units.inchesToMeters(315.5),
			tapeWidth = Units.inchesToMeters(2.0);

		// Dimensions for community and charging station, including the tape.
		public static final class Community {

			// Region dimensions
			public static final double
				innerX = 0.0,
				midX = Units.inchesToMeters(132.375), // Tape to the left of charging station
				outerX = Units.inchesToMeters(193.25), // Tape to the right of charging station
				leftY = Grids.nodeY[8] + Units.inchesToMeters(20.19),
				midY = leftY - Units.inchesToMeters(59.39) + tapeWidth,
				rightY = 0.0;
			public static final Translation2d[] regionCorners =
				new Translation2d[] {
					new Translation2d(innerX, rightY),
					new Translation2d(innerX, leftY),
					new Translation2d(midX, leftY),
					new Translation2d(midX, midY),
					new Translation2d(outerX, midY),
					new Translation2d(outerX, rightY),
				};

			// Charging station dimensions
			public static final double
				chargingStationInnerX = Grids.outerX + Units.inchesToMeters(60.69),
				chargingStationOuterX = outerX - tapeWidth,
				chargingStationLeftY = midY - tapeWidth,
				chargingStationRightY = Units.inchesToMeters(59.39),
				chargingStationLength = chargingStationOuterX - chargingStationInnerX,
				chargingStationWidth = chargingStationLeftY - chargingStationRightY;
			public static final Translation2d[] chargingStationCorners =
				new Translation2d[] {
					new Translation2d(chargingStationInnerX, chargingStationRightY),
					new Translation2d(chargingStationInnerX, chargingStationLeftY),
					new Translation2d(chargingStationOuterX, chargingStationRightY),
					new Translation2d(chargingStationOuterX, chargingStationLeftY)
				};

			// Cable bump
			public static final double
				cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25),
				cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
			public static final Translation2d[] cableBumpCorners =
				new Translation2d[] {
					new Translation2d(cableBumpInnerX, 0.0),
					new Translation2d(cableBumpInnerX, chargingStationRightY),
					new Translation2d(cableBumpOuterX, 0.0),
					new Translation2d(cableBumpOuterX, chargingStationRightY)
				};

		}

		// Dimensions for grids and nodes
		public static final class Grids {

			// X layout
			public static final double
				outerX = Units.inchesToMeters(54.25),
				lowX = outerX - (Units.inchesToMeters(14.25) / 2.0), // Centered when under cube nodes
				midX = outerX - Units.inchesToMeters(22.75),
				highX = outerX - Units.inchesToMeters(39.75);

			// Y layout
			public static final int nodeRowCount = 9;
			public static final double[] nodeY =
				new double[] {
					Units.inchesToMeters(20.19 + 22.0 * 0),
					Units.inchesToMeters(20.19 + 22.0 * 1),
					Units.inchesToMeters(20.19 + 22.0 * 2),
					Units.inchesToMeters(20.19 + 22.0 * 3),
					Units.inchesToMeters(20.19 + 22.0 * 4),
					Units.inchesToMeters(20.19 + 22.0 * 5),
					Units.inchesToMeters(20.19 + 22.0 * 6),
					Units.inchesToMeters(20.19 + 22.0 * 7),
					Units.inchesToMeters(20.19 + 22.0 * 8)
				};

			// Z layout
			public static final double
				cubeEdgeHigh = Units.inchesToMeters(3.0),
				highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh,
				midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh,
				highConeZ = Units.inchesToMeters(46.0),
				midConeZ = Units.inchesToMeters(34.0);

			// Translations (all nodes in the same column/row have the same X/Y coordinate)
			public static final Translation2d[]
				lowTranslations = new Translation2d[nodeRowCount],
				midTranslations = new Translation2d[nodeRowCount],
				highTranslations = new Translation2d[nodeRowCount];
			public static final Translation3d[]
				low3dTranslations = new Translation3d[nodeRowCount],
				mid3dTranslations = new Translation3d[nodeRowCount],
				high3dTranslations = new Translation3d[nodeRowCount];

			static {
				for (int i = 0; i < nodeRowCount; i++) {
					boolean isCube = i == 1 || i == 4 || i == 7;
					lowTranslations[i] = new Translation2d(lowX, nodeY[i]);
					low3dTranslations[i] = new Translation3d(lowX, nodeY[i], 0.0);
					midTranslations[i] = new Translation2d(midX, nodeY[i]);
					mid3dTranslations[i] = new Translation3d(midX, nodeY[i], isCube ? midCubeZ : midConeZ);
					highTranslations[i] = new Translation2d(highX, nodeY[i]);
					high3dTranslations[i] = new Translation3d(highX, nodeY[i], isCube ? highCubeZ : highConeZ);
				}
			}

			// Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
			public static final double
				complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0, // Centered X under cone nodes
				complexLowXCubes = lowX, // Centered X under cube nodes
				complexLowOuterYOffset = nodeY[0] - (Units.inchesToMeters(3.0) + (Units.inchesToMeters(25.75) / 2.0));

			public static final Translation2d[] complexLowTranslations =
				new Translation2d[] {
					new Translation2d(complexLowXCones, nodeY[0] - complexLowOuterYOffset),
					new Translation2d(complexLowXCubes, nodeY[1]),
					new Translation2d(complexLowXCones, nodeY[2]),
					new Translation2d(complexLowXCones, nodeY[3]),
					new Translation2d(complexLowXCubes, nodeY[4]),
					new Translation2d(complexLowXCones, nodeY[5]),
					new Translation2d(complexLowXCones, nodeY[6]),
					new Translation2d(complexLowXCubes, nodeY[7]),
					new Translation2d(complexLowXCones, nodeY[8] + complexLowOuterYOffset),
				};

			public static final Translation3d[] complexLow3dTranslations =
				new Translation3d[] {
					new Translation3d(complexLowXCones, nodeY[0] - complexLowOuterYOffset, 0.0),
					new Translation3d(complexLowXCubes, nodeY[1], 0.0),
					new Translation3d(complexLowXCones, nodeY[2], 0.0),
					new Translation3d(complexLowXCones, nodeY[3], 0.0),
					new Translation3d(complexLowXCubes, nodeY[4], 0.0),
					new Translation3d(complexLowXCones, nodeY[5], 0.0),
					new Translation3d(complexLowXCones, nodeY[6], 0.0),
					new Translation3d(complexLowXCubes, nodeY[7], 0.0),
					new Translation3d(complexLowXCones, nodeY[8] + complexLowOuterYOffset, 0.0),
				};

		}

		// Dimensions for loading zone and substations, including the tape
		public static final class LoadingZone {

			// Region dimensions
			public static final double
				width = Units.inchesToMeters(99.0),
				innerX = Field.FIELD_LENGTH,
				midX = FIELD_LENGTH - Units.inchesToMeters(132.25),
				outerX = FIELD_LENGTH - Units.inchesToMeters(264.25),
				leftY = Field.FIELD_WIDTH,
				midY = leftY - Units.inchesToMeters(50.5),
				rightY = leftY - width;
			public static final Translation2d[] regionCorners =
				new Translation2d[] {
					new Translation2d(midX, rightY), // Start at lower left next to border with opponent community
					new Translation2d(midX, midY),
					new Translation2d(outerX, midY),
					new Translation2d(outerX, leftY),
					new Translation2d(innerX, leftY),
					new Translation2d(innerX, rightY),
				};

			// Double substation dimensions
			public static final double
				doubleSubstationLength = Units.inchesToMeters(14.0),
				doubleSubstationX = innerX - doubleSubstationLength,
				doubleSubstationShelfZ = Units.inchesToMeters(37.375),
				doubleSubstationCenterY = FIELD_WIDTH - Units.inchesToMeters(49.76);

			// Single substation dimensions
			public static final double
				singleSubstationWidth = Units.inchesToMeters(22.75),
				singleSubstationLeftX = Field.FIELD_LENGTH - doubleSubstationLength - Units.inchesToMeters(88.77),
				singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0),
				singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
			public static final Translation2d singleSubstationTranslation =
				new Translation2d(singleSubstationCenterX, leftY);

			public static final double
				singleSubstationHeight = Units.inchesToMeters(18.0),
				singleSubstationLowZ = Units.inchesToMeters(27.125),
				singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0),
				singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;

		}

		// Locations of staged game pieces
		public static final class StagingLocations {

			public static final double
				centerOffsetX = Units.inchesToMeters(47.36),
				positionX = FIELD_LENGTH / 2.0 - Units.inchesToMeters(47.36),
				firstY = Units.inchesToMeters(36.19),
				separationY = Units.inchesToMeters(48.0);
			public static final Translation2d[] translations = new Translation2d[4];

			static {
				for (int i = 0; i < translations.length; i++) {
					translations[i] = new Translation2d(positionX, firstY + (i * separationY));
				}
			}

		}

		// AprilTag constants
		/*
		public static final double aprilTagWidth = Units.inchesToMeters(6.0);
		public static final AprilTagFieldLayout aprilTags =
			new AprilTagFieldLayout(
				List.of(
					new AprilTag(
						1,
						new Pose3d(
							Units.inchesToMeters(610.77),
							Grids.nodeY[1],
							Units.inchesToMeters(18.22),
							new Rotation3d(0.0, 0.0, Math.PI))),
					new AprilTag(
						2,
						new Pose3d(
							Units.inchesToMeters(610.77),
							Grids.nodeY[4],
							Units.inchesToMeters(18.22),
							new Rotation3d(0.0, 0.0, Math.PI))),
					new AprilTag(
						3,
						new Pose3d(
							Units.inchesToMeters(610.77),
							Grids.nodeY[7],
							Units.inchesToMeters(18.22),
							new Rotation3d(0.0, 0.0, Math.PI))),
					new AprilTag(
						4,
						new Pose3d(
							Units.inchesToMeters(636.96),
							LoadingZone.doubleSubstationCenterY,
							Units.inchesToMeters(27.38),
							new Rotation3d(0.0, 0.0, Math.PI))),
					new AprilTag(
						5,
						new Pose3d(
							Units.inchesToMeters(14.25),
							LoadingZone.doubleSubstationCenterY,
							Units.inchesToMeters(27.38),
							new Rotation3d())),
					new AprilTag(
						6,
						new Pose3d(
							Units.inchesToMeters(40.45),
							Grids.nodeY[7],
							Units.inchesToMeters(18.22),
							new Rotation3d())),
					new AprilTag(
						7,
						new Pose3d(
							Units.inchesToMeters(40.45),
							Grids.nodeY[4],
							Units.inchesToMeters(18.22),
							new Rotation3d())),
					new AprilTag(
						8,
						new Pose3d(
							Units.inchesToMeters(40.45),
							Grids.nodeY[1],
							Units.inchesToMeters(18.22),
							new Rotation3d()))),
				fieldLength,
				fieldWidth);
		*/


	}


}