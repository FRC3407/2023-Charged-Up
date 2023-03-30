package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;


public final class Manipulator implements Sendable {
    
    public static final class Arm implements Subsystem, Sendable {

		/* TODO:
		 * Confirm limit switch implementation (FWD vs REV)
		 * Decide on arm angle directionality and homing/starting location conventions
		 * Confirm encoder inversions
		 * Configure PIDF
		 */

		public static final LimitSwitchSource LIMIT_SWITCH_SOURCE = LimitSwitchSource.FeedbackConnector;
		public static final LimitSwitchNormal LIMIT_SWITCH_NORMALITY = LimitSwitchNormal.NormallyOpen;
		public static final FeedbackDevice WINCH_FEEDBACK_TYPE = FeedbackDevice.Analog;
		public static final int
			FB_UNITS_PER_ROTATION = Constants.ANALOG_POT_UNITS_PER_REVOLUTION,
			CONTROL_LOOP_IDX = 0;
		public static final boolean
			INVERT_ARM_ANGLE_ENCODER = false,
			CLEAR_ANGLE_ON_BOTTOM = false,
			CLEAR_ANGLE_ON_TOP = false;

        private final WPI_TalonSRX winch;
		// private final WPI_TalonSRX extender;

        public Arm(int id) {
            this.winch = new WPI_TalonSRX(id);

            this.winch.configFactoryDefault();
            this.winch.configSelectedFeedbackSensor(WINCH_FEEDBACK_TYPE, CONTROL_LOOP_IDX, 0);
            //this.winch.setSelectedSensorPosition(0.0, CONTROL_LOOP_IDX, 0);      // if the potentiometer is absolute, then we probably want the absolute value right?
			this.winch.setSensorPhase(INVERT_ARM_ANGLE_ENCODER);
			this.winch.setNeutralMode(NeutralMode.Brake);
			this.winch.configForwardLimitSwitchSource(LIMIT_SWITCH_SOURCE, LIMIT_SWITCH_NORMALITY);
			this.winch.configReverseLimitSwitchSource(LIMIT_SWITCH_SOURCE, LIMIT_SWITCH_NORMALITY);
			this.winch.configClearPositionOnLimitF(CLEAR_ANGLE_ON_TOP, 0);
			this.winch.configClearPositionOnLimitR(CLEAR_ANGLE_ON_BOTTOM, 0);
			this.winch.config_kF(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KF);
			this.winch.config_kP(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KP);
			this.winch.config_kI(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KI);
			this.winch.config_kD(CONTROL_LOOP_IDX, Constants.ARM_ANGLE_KD);
			this.winch.configMotionAcceleration(
				Constants.ARM_ANGLE_ACC_DEG_PER_SEC_SQRD / 360.0 * FB_UNITS_PER_ROTATION / 10.0);
			this.winch.configMotionCruiseVelocity(
				Constants.ARM_ANGLE_CRUISE_DEG_PER_SEC / 360.0 * FB_UNITS_PER_ROTATION / 10.0);
			// also see: config_IntegralZone(), configClosedLoopPeakOutput(), setStatusFramePeriod(), etc...
        }

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Arm Encoder Value Raw", this::getWinchRawPosition, null);
			b.addDoubleProperty("Arm Angle (degrees)", this::getWinchDegPosition, null);
			b.addDoubleProperty("Arm Angle Rate", this::getWinchDegVelocity, null);
			b.addIntegerProperty("Winch Lower Limit", this.winch::isRevLimitSwitchClosed, null);
			b.addIntegerProperty("Winch Upper Limit", this.winch::isFwdLimitSwitchClosed, null);
			b.addDoubleProperty("Winch Voltage", this.winch::getMotorOutputVoltage, null);
			b.addDoubleArrayProperty("Winch Current [In:Out]",
				()->{ return new double[]{
					this.winch.getSupplyCurrent(),
					this.winch.getStatorCurrent()
				}; }, null);
			b.addDoubleProperty("Winch MC Temp", this.winch::getTemperature, null);
		}

		public void setWinchVoltage(double v) {
			this.winch.setVoltage(v);
		}
		public void setWinchPosition(double deg) {
			this.winch.set(ControlMode.Position,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void setWinchPosition_MM(double deg) {
			this.winch.set(ControlMode.MotionMagic,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void resetPosition() {
			this.winch.setSelectedSensorPosition(0.0);
		}

		public double getWinchRawPosition() {
			return this.winch.getSelectedSensorPosition(CONTROL_LOOP_IDX);
		}
		public double getWinchRawVelocity() {
			return this.winch.getSelectedSensorVelocity(CONTROL_LOOP_IDX);
		}

		public double getWinchRotPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_ROTATION;
		}
		public double getWinchDegPosition() {
			return this.getWinchRawPosition() / FB_UNITS_PER_ROTATION * 360.0;
		}
		public double getWinchRotVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION;
		}
		public double getWinchDegVelocity() {
			return this.getWinchRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION * 360;
		}


    }
    public static final class Grabber implements Subsystem, Sendable {

		/* TODO:
		 * Determine grabber angle convention
		 * Determine homing location if any, soft limits, etc...
		 * Configure PIDF
		 */

		public static final double
			WRIST_MIN_PULSE_uS = 0.5,
			WRIST_MAX_PULSE_uS = 2.5,
			WRIST_TOTAL_RANGE = 270.0,
			WRIST_CENTER_OFFSET = 135,	// should make setting wrist angle simpler if angles are based off of center (parallel to arm) angle
			WRIST_LOWER_LIMIT = -135,
			WRIST_UPPER_LIMIT = 135,
			WRIST_LOWER_LIMIT_PERCENT = (WRIST_LOWER_LIMIT + WRIST_CENTER_OFFSET) / WRIST_TOTAL_RANGE,
			WRIST_UPPER_LIMIT_PERCENT = (WRIST_UPPER_LIMIT + WRIST_CENTER_OFFSET) / WRIST_TOTAL_RANGE,
			GRAB_MAX_ANGLE = 110;
		public static final int
			FB_UNITS_PER_ROTATION = (int)(Constants.NEVEREST_UNITS_PER_REVOLUTION * Constants.GRABBER_GEARING_IN2OUT),
			CONTROL_LOOP_IDX = 0;
		public static final boolean
			INVERT_GRAB_ENCODER = false;

        private final WPI_TalonSRX main;
        private final Servo wrist;

        public Grabber(int id, int schan) {
            this.main = new WPI_TalonSRX(id);
            this.wrist = new Servo(schan);

            this.wrist.setBounds(WRIST_MAX_PULSE_uS, 0, 0, 0, WRIST_MIN_PULSE_uS);
			this.main.configFactoryDefault();
			this.main.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, CONTROL_LOOP_IDX, 0);
			this.main.setSelectedSensorPosition(0.0, CONTROL_LOOP_IDX, 0);
			this.main.setSensorPhase(INVERT_GRAB_ENCODER);
			this.main.configForwardSoftLimitThreshold(GRAB_MAX_ANGLE / 360.0 * FB_UNITS_PER_ROTATION);		// 110 degrees max
			this.main.configReverseSoftLimitThreshold(0.0);								// dont let it go backwards
			this.main.configForwardSoftLimitEnable(false);
			this.main.configReverseSoftLimitEnable(false);
			this.main.config_kF(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KF);
			this.main.config_kP(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KP);
			this.main.config_kI(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KI);
			this.main.config_kD(CONTROL_LOOP_IDX, Constants.GRAB_POSITION_KD);
        }

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Wrist Angle", this::getWristAngle, null);
			b.addDoubleProperty("Wrist Percent Output", this::getWristPercent, null);
			b.addDoubleProperty("Grabber Rotation (degrees)", this::getGrabDegPosition, null);
			b.addDoubleProperty("Grabber Encoder Raw", this::getGrabRawPosition, null);
			b.addDoubleProperty("Grabber Rotation Rate", this::getGrabDegVelocity, null);
			b.addDoubleProperty("Grabber Width (inches)", this::getGrabWidth, null);
			b.addDoubleProperty("Grab Motor Voltage", this.main::getMotorOutputVoltage, null);
			b.addDoubleArrayProperty("Grab Motor Current [In:Out]", ()->{
				return new double[]{
					this.main.getSupplyCurrent(),
					this.main.getStatorCurrent()
				}; }, null);
			b.addDoubleProperty("Grab MC Temp", this.main::getTemperature, null);
		}

		public static double grabAngleToWidth(double deg) {
			return 2.0 * (Constants.GRABBER_PIVOT_OFFSET_INCHES +
				Constants.GRABBER_ROT_RADIUS_INCHES * Math.sin(Math.toRadians(deg)) -
					Constants.GRABBER_FINGER_OFFSET_INCHES);
		}
		public static double grabWidthToAngle(double inches) {
			double ratio = (((inches / 2.0) + Constants.GRABBER_FINGER_OFFSET_INCHES -
				Constants.GRABBER_PIVOT_OFFSET_INCHES) / Constants.GRABBER_ROT_RADIUS_INCHES);
			if(ratio >= 1.0) { return 90.0; }
			return Math.toDegrees(Math.asin(ratio));
		}

		public void setGrabberVoltage(double v) {
			this.main.setVoltage(v);
		}
		public void setGrabberAngle(double deg) {	// also probably account for W0 being not perfectly at 0 degrees
			this.main.set(ControlMode.Position,
				deg / 360.0 * FB_UNITS_PER_ROTATION);
		}
		public void setGrabberWidth(double inches) {	// also probably account for A0 being not perfectly at 0 inches
			this.main.set(ControlMode.Position,
				grabWidthToAngle(inches) / 360.0 * FB_UNITS_PER_ROTATION);
		}

		public void setWristPercent(double p) {
			p = p > WRIST_UPPER_LIMIT_PERCENT ? WRIST_UPPER_LIMIT_PERCENT : (p < WRIST_LOWER_LIMIT_PERCENT ? WRIST_LOWER_LIMIT_PERCENT : p);
			this.wrist.setPosition(p);
		}
		public void setWristAngle(double deg) {
			deg += WRIST_CENTER_OFFSET;
			if(deg < 0) { deg = 0; }
			if(deg > WRIST_TOTAL_RANGE) { deg = WRIST_TOTAL_RANGE; }
			this.setWristPercent(deg / WRIST_TOTAL_RANGE);
		}

		public double getWristPercent() {
			return this.wrist.getPosition();
		}
		public double getWristAngle() {
			return this.wrist.getPosition() * WRIST_TOTAL_RANGE - WRIST_CENTER_OFFSET;
		}

		public double getGrabRawPosition() {
			return this.main.getSelectedSensorPosition(CONTROL_LOOP_IDX);
		}
		public double getGrabRawVelocity() {
			return this.main.getSelectedSensorVelocity(CONTROL_LOOP_IDX);
		}
		public double getGrabRotPosition() {
			return this.getGrabRawPosition() / FB_UNITS_PER_ROTATION;
		}
		public double getGrabDegPosition() {
			return this.getGrabRawPosition() / FB_UNITS_PER_ROTATION * 360.0;
		}
		public double getGrabWidth() {
			return grabAngleToWidth(this.getGrabDegPosition());
		}
		public double getGrabRotVelocity() {
			return this.getGrabRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION;
		}
		public double getGrabDegVelocity() {
			return this.getGrabRawVelocity() * 10.0 / FB_UNITS_PER_ROTATION * 360.0;
		}

    }



    public final Arm arm;
    public final Grabber grabber;

    public Manipulator(Arm a, Grabber g) {
        this.arm = a;
        this.grabber = g;
    }

	@Override
	public void initSendable(SendableBuilder b) {

	}

	public void startLogging(String basekey) {
		SmartDashboard.putData(basekey, this);
		SmartDashboard.putData(basekey + "/Arm", this.arm);
		SmartDashboard.putData(basekey + "/Grabber", this.grabber);
	}





	public static class TestManipulator extends CommandBase {

		public static final double
			ARM_WINCH_VOLTAGE_SCALE = 5.0,
			GRAB_CLAW_VOLTAGE_SCALE = 7.0;

		private final Manipulator
			manipulator;
		private final DoubleSupplier
			arm_percent,
			grab_percent,
			wrist_set;
		private double wristPosition = 0.5; 

		public TestManipulator(Manipulator m, DoubleSupplier a, DoubleSupplier g, DoubleSupplier w) {
			this.manipulator = m;
			this.arm_percent = a;
			this.grab_percent = g;
			this.wrist_set = w;
		}

		private double calcWristPos()
		{
			if(this.wrist_set.getAsDouble() > 0 && wristPosition < 1.0)
			{
				if(wristPosition + 0.001 < 0.95)
				{
					return wristPosition += 0.001;
				}
				else
				{
					return this.WRIST_UPPER_LIMIT_PERCENT - 0.1;
				}
				
			}
			else if(this.wrist_set.getAsDouble() < 0 && wristPosition > 0.0)
			{
				if(wristPosition- 0.001 > 0.05)
				{
					return wristPosition -= 0.001;
				}
				else 
				{
					return this.WRIST_UPPER_LIMIT_PERCENT + 0.1;
				}
				
			}
			else 
			{
				return wristPosition;
			}
		}

		@Override
		public void initialize() {
			// System.out.println("Starting manipulator test!");
		}
		@Override
		public void execute() {
			this.manipulator.arm.setWinchVoltage(this.arm_percent.getAsDouble() * ARM_WINCH_VOLTAGE_SCALE);
			this.manipulator.grabber.setGrabberVoltage(this.grab_percent.getAsDouble() * GRAB_CLAW_VOLTAGE_SCALE);
			double p = calcWristPos();
			this.manipulator.grabber.setWristPercent(p);
			System.out.println("Wrist position =" + p);
		}
		@Override
		public boolean isFinished() {
			return false;
		}
		@Override
		public void end(boolean isfinished) {
			this.manipulator.arm.setWinchVoltage(0);
			this.manipulator.grabber.setGrabberVoltage(0);
			this.manipulator.grabber.setWristPercent(0);
		}

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Winch Voltage Setpoint", ()->this.arm_percent.getAsDouble()*ARM_WINCH_VOLTAGE_SCALE, null);
			b.addDoubleProperty("Grabber Voltage Setpoint", ()->this.grab_percent.getAsDouble()*GRAB_CLAW_VOLTAGE_SCALE, null);
			b.addDoubleProperty("Wrist Position Setpoint", ()->this.wrist_set.getAsDouble(), null);
		}

	}



    // public static class OperateClaw extends CommandBase {

    //     private final Grabber grabber;
    //     private final BooleanSupplier opening, closing;
    //     private final double volts_output;

    //     public OperateClaw(Grabber g, BooleanSupplier o, BooleanSupplier c, double volts_out) {
    //         this.grabber = g;
    //         this.opening = o;
    //         this.closing = c;
    //         this.volts_output = volts_out;
    //     }

    //     @Override
    //     public void initialize() {

    //     }
    //     @Override
    //     public void execute() {
    //         if(this.opening.getAsBoolean()) {   // and not opened too wide
    //             // open
    //         } else if(this.closing.getAsBoolean()) {    // and not already completely closed
    //             // close
    //         } else {
    //             // stop voltage
    //         }
    //     }
    //     @Override
    //     public boolean isFinished() {
    //         return false;
    //     }
    //     @Override
    //     public void end(boolean isfinished) {
    //         // stop 
    //     }

    // }


}
