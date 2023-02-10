package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.Sendable;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.team3407.Input.AnalogSupplier;
import frc.robot.team3407.drive.Types.*;


public class DriveBase extends MotorSafety implements Subsystem, Sendable {

    public static class ClosedLoopParams {
        public final Inversions
            encoder_inversions;
        public final double
            track_width_meters,
            wheel_diameter_meters,

			static_voltage,					    // kS -- drive motor feedforward term -- "voltage to overcome static friction" -- volts
			volt_seconds_per_meter,			    // kV -- drive motor feedforward term -- "voltage to hold a given velocity" -- volts * seconds / meters
			volt_seconds_sqrd_per_meter,	    // kA -- drive motor feedforward term -- "voltage to hold an acceleration" -- volts * seconds^2 / meters
			volt_seconds_per_meter_gain,	    // kP -- proportional feedback gain -- "voltage applied per unit velocity error" -- volts * seconds / meters
            volts_per_meter_gain,               // kI -- integral feedback gain -- "voltage applied per unit displacement error" -- volts / meters
            volt_seconds_sqrd_per_meter_gain,   // kD -- derivitive feedback gain -- "voltage applied per unit acceleration error" -- volts * seconds^2 / meters

			max_voltage_output,				// maximum voltage to be applied to the drivebase motors -- volts
			max_velocity,				    // maximum velocity of either side -- meters / second
			max_acceleration    			// maximum acceleration of either side -- meters / second^2
        ;
        public ClosedLoopParams(
            double trackwidth, double wheeldiameter,
            double kS, double kV, double kA,
            double kP, double kI, double kD,
            double max_volts, double max_vel, double max_acc,
            Inversions encoderinversions
        ) {
            this.encoder_inversions = encoderinversions;
            this.track_width_meters = trackwidth;
            this.wheel_diameter_meters = wheeldiameter;
            this.static_voltage = kS;
            this.volt_seconds_per_meter = kV;
            this.volt_seconds_sqrd_per_meter = kA;
            this.volt_seconds_per_meter_gain = kP;
            this.volts_per_meter_gain = kI;
            this.volt_seconds_sqrd_per_meter_gain = kD;
            this.max_voltage_output = max_volts;
            this.max_velocity = max_vel;
            this.max_acceleration = max_acc;
        }
        public double kS() { return this.static_voltage; }
		public double kV() { return this.volt_seconds_per_meter; }
		public double kA() { return this.volt_seconds_sqrd_per_meter; }
		public double kP() { return this.volt_seconds_per_meter_gain; }
        public double kI() { return this.volts_per_meter_gain; }
        public double kD() { return this.volt_seconds_sqrd_per_meter_gain; }
        public SimpleMotorFeedforward getFeedforward() {
			return new SimpleMotorFeedforward(
				this.kS(),
				this.kV(),
				this.kA()
			);
		}
        public PIDController getFeedbackController() {	// should probably be a 'generator' rather than a 'getter'
            return new PIDController(this.kP(), this.kI(), this.kD());
        }
        public ProfiledPIDController getProfiledFeedbackController() {
            return new ProfiledPIDController(
                this.kP(), this.kI(), this.kD(),
                new TrapezoidProfile.Constraints(this.max_velocity, this.max_acceleration)
            );
        }
    }
    
    private final Gyro 
        gyro;
    private final WPI_TalonSRX
        left, left2,
        right, right2;
    private final ClosedLoopParams
        parameters;

    private final SimpleMotorFeedforward feedforward;
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    private Pose2d elapsed_cache;

    public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params) {
        this.parameters = params;
        this.gyro = gy;
        this.left = map.front_left;
        this.right = map.front_right;
        this.left2 = map.back_left;
        this.right2 = map.back_right;

        this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d(), 0, 0);
        this.kinematics = new DifferentialDriveKinematics(this.parameters.track_width_meters);
        this.feedforward = this.parameters.getFeedforward();

        this.left.configFactoryDefault();
        this.right.configFactoryDefault();
        this.left2.configFactoryDefault();
        this.right2.configFactoryDefault();
        this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
		this.left.setSensorPhase(parameters.encoder_inversions.left);
		this.right.setSensorPhase(parameters.encoder_inversions.right);
        this.left2.follow(this.left);
        this.right2.follow(this.right);
        this.left2.setInverted(InvertType.FollowMaster);
        this.right2.setInverted(InvertType.FollowMaster);
    }

    @Override
    public void periodic() {
        this.odometry.update(
			this.gyro.getRotation2d(),
			this.getLeftPosition(),
			this.getRightPosition()
		);
    }
    @Override
    public void initSendable(SendableBuilder b) {
        b.setSmartDashboardType("DriveBase [4x TalonSRX Differential]");
        b.addDoubleProperty("Left Distance", this::getLeftPosition, null);
		b.addDoubleProperty("Right Distance", this::getRightPosition, null);
		b.addDoubleProperty("Left Velocity", this::getLeftVelocity, null);
		b.addDoubleProperty("Right Velocity", this::getRightVelocity, null);
		b.addDoubleProperty("Rotation (Total)", this::getContinuousAngle, null);
        b.addDoubleArrayProperty("Output Voltage [L1, R1, L2, R2]",
            ()->{ return new double[]{
                this.left.getMotorOutputVoltage(),
                this.right.getMotorOutputVoltage(),
                this.left2.getMotorOutputVoltage(),
                this.right2.getMotorOutputVoltage()
            }; }, null);
        b.addDoubleArrayProperty("Output Current [L1, R1, L2, R2]",
            ()->{ return new double[]{
                this.left.getStatorCurrent(),
                this.right.getStatorCurrent(),
                this.left2.getStatorCurrent(),
                this.right2.getStatorCurrent()
            }; }, null);
    }
    @Override
    public void stopMotor() {
        this.left.stopMotor();
        this.left2.stopMotor();
        this.right.stopMotor();
        this.right2.stopMotor();
        super.feed();
    }
    @Override
    public String getDescription() {
        return "DriveBase [2023]";
    }

    public Field2d getSendableLocation() {
        Field2d map = new Field2d();
        SmartDashboard.putData("Robot Location", map);
        return map;
    }
    public void updateFieldLocation(Field2d map) {
        map.setRobotPose(this.getTotalPose());
    }


    /** Get a tankdrive command, controlled by inputs that return input percentages (ei. joystick axis outputs: -1 to 1 )
     * @param l a -1 to 1 ranged input supplier for the left side
     * @param r a -1 to 1 ranged input supplier for the right side
     * @return A command for driving the drivebase using tankdrive
     */
    public Command tankDrivePercent(DoubleSupplier l, DoubleSupplier r) {
        return new TankDrivePercent(this, l, r);
    }
    /** Get a tankdrive command, controlled by inputs that return voltages for each side
     * @param lvt a ~-12 to ~12 ranged input supplier for the left side motor voltages
     * @param rvt a ~-12 to ~12 ranged input supplier for the right side motor voltages
     * @return A command for driving the drivebase using tankdrive
     */
    public Command tankDriveVoltage(DoubleSupplier lvt, DoubleSupplier rvt) {
        return new TankDriveVoltage(this, lvt, rvt);
    }
    /** Get a tankdrive command, controlled by inputs that provide target velocities for each side
     * @param lv a velocity supplier in METERS PER SECOND for the left side
     * @param rv a velocity supplier in METERS PER SECOND for the right side
     * @return A command for driving the drivebase using tankdrive
     */
    public Command tankDriveVelocity(DoubleSupplier lv, DoubleSupplier rv) {
        return new TankDriveVelocity(this, lv, rv);
    }
    /** Get a tankdrive command, controlled by inputs that provide target velocities for each side -- transitions are limited by a trapazoid profile generator
     * @param lv a velocity supplier in METERS PER SECOND for the left side
     * @param rv a velocity supplier in METERS PER SECOND for the right side
     * @return A command for driving the drivebase using tankdrive
     */
    public Command tankDriveVelocityProfiled(DoubleSupplier lv, DoubleSupplier rv) {
        return new TankDriveVelocity_P(this, lv, rv);
    }
    /** Get a active parking command - a routine where the robot attempts to stay in the same position using the encoder position and a negative feedback p-loop
     * @param p_gain The proportional gain in volts/meter that the robot will apply when any position error is accumulated
     * @return A command for active parking
     */
    public Command activePark(double p_gain) {
        return new ActivePark(this, p_gain);
    }


    public void setDriveVoltage(double lv, double rv) {
        this.left.setVoltage(lv);
        this.right.setVoltage(rv);
        super.feed();
    }
    public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
    public void zeroHeading() {
		this.gyro.reset();
	}
    public void resetOdometry(Pose2d p) {
		this.resetEncoders();
		this.elapsed_cache = this.getTotalPose();
		this.odometry.resetPosition(
            this.getRotation(), 0.0, 0.0, p);
	}

    public void setCoastMode() {
        this.left.setNeutralMode(NeutralMode.Coast);
        this.left2.setNeutralMode(NeutralMode.Coast);
        this.right.setNeutralMode(NeutralMode.Coast);
        this.right2.setNeutralMode(NeutralMode.Coast);
    }
    public void setBrakeMode() {
        this.left.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);
        this.right.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);
    }


    public double getRawLeftPosition() {
		return this.left.getSelectedSensorPosition();
	}
	public double getRawRightPosition() {
		return this.right.getSelectedSensorPosition();
	}
	public double getRawLeftVelocity() {
		return this.left.getSelectedSensorVelocity();
	}
	public double getRawRightVelocity() {
		return this.right.getSelectedSensorVelocity();
	}

    /* !!!--> IN METERS AND METERS/SECOND <--!!! */
    public double getLeftPosition() {
		return this.getRawLeftPosition()				    // output is in encoder units...
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION		// to get total rotations
			* this.parameters.wheel_diameter_meters * Math.PI;	// to get total distance
	}
	public double getRightPosition() {
		return this.getRawRightPosition()				    // ^^^
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION
			* this.parameters.wheel_diameter_meters * Math.PI;
	}
	public double getLeftVelocity() {
		return this.getRawLeftVelocity()				    // output is in encoder units per 100 ms
			* 10											// to get encoder units per second
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION		// to get rotations per second
			* this.parameters.wheel_diameter_meters * Math.PI;	// to get meters per second
	}
	public double getRightVelocity() {
		return this.getRawRightVelocity()				    // ^^^
			* 10
			/ Constants.SRX_MAG_UNITS_PER_REVOLUTION
			* this.parameters.wheel_diameter_meters * Math.PI;
	}
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());
	}

    public double getContinuousAngle() {	// in degrees
		return this.gyro.getAngle();
	}
	public double getHeading() {			// from -180 to 180
		return this.gyro.getRotation2d().getDegrees();
	}
	public double getTurnRate() {	        // in degrees per second
		return -this.gyro.getRate();
	}
	public Rotation2d getRotation() {
		return this.gyro.getRotation2d();
	}

    public Pose2d getDeltaPose() {
        return this.odometry.getPoseMeters();
    }
    public Pose2d getTotalPose() {
        Pose2d current = this.getDeltaPose();
        return this.elapsed_cache.plus(new Transform2d(current.getTranslation(), current.getRotation()));
    }

    public DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return new DifferentialDriveVoltageConstraint(
			this.feedforward, this.kinematics, this.parameters.max_voltage_output
		);
	}
	public TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(
			this.parameters.max_velocity,
			this.parameters.max_acceleration
		).setKinematics(
			this.kinematics
		).addConstraint(
			this.getVoltageConstraint()
		);
	}



	/** TankDrive that is controlled by 2 'voltage-returning' analog suppliers */
    public static class TankDriveVoltage extends CommandBase {
        
        private final DriveBase drivebase;
        private final DoubleSupplier left, right;

        public TankDriveVoltage(DriveBase db, DoubleSupplier lv, DoubleSupplier rv) {
            this.drivebase = db;
            this.left = lv;
            this.right = rv;
            super.addRequirements(db);
        }

        @Override
        public void initialize() {

        }
        @Override
        public void execute() {
            this.drivebase.setDriveVoltage(
                this.left.getAsDouble(),
                this.right.getAsDouble()
            );
        }
        @Override
        public boolean isFinished() {
            return false;
        }
        @Override
        public void end(boolean interrupted) {
            this.drivebase.setDriveVoltage(0, 0);
        }


    }
	/** TankDrive that is controlled by 2 'percent-returning' analog suppliers -- ex. joystick axis */
	public static class TankDrivePercent extends TankDriveVoltage {

		public TankDrivePercent(DriveBase db, DoubleSupplier l, DoubleSupplier r) {
			super(db, l, r);
		}

		@Override
		public void execute() {
			super.drivebase.setDriveVoltage(
				super.left.getAsDouble() * super.drivebase.parameters.max_voltage_output,
				super.right.getAsDouble() * super.drivebase.parameters.max_voltage_output
			);
		}


	}
	/** TankDrive that is controlled by 2 'velocity-returning' analog suppliers -- **METERS PER SECOND** */
    public static class TankDriveVelocity extends CommandBase {

        private final DriveBase drivebase;
        private final DoubleSupplier left, right;
        private final PIDController left_fb, right_fb;	// feedback controllers for left and

        public TankDriveVelocity(
            DriveBase db, DoubleSupplier l, DoubleSupplier r
        ) {
            this.drivebase = db;
            this.left = l;
            this.right = r;
            this.left_fb = db.parameters.getFeedbackController();
            this.right_fb = db.parameters.getFeedbackController();
            super.addRequirements(db);
        }

        @Override
        public void initialize() {
			this.left_fb.reset();
			this.right_fb.reset();
		}
        @Override
        public void execute() {
			double
				lt = this.left.getAsDouble(),	// the target velocity from the left input --> METERS PER SECOND
				rt = this.right.getAsDouble(),	// ^^^ for the right side
				lc = this.drivebase.getLeftVelocity(),  // the actual velocity
				rc = this.drivebase.getRightVelocity();
            this.drivebase.setDriveVoltage(
				this.drivebase.feedforward.calculate(lt) +  // the calculated feedforward
					this.left_fb.calculate(lc, lt),   		// add the feedback adjustment
				this.drivebase.feedforward.calculate(rt) +
					this.right_fb.calculate(rc, rt)
			);
        }
		@Override
		public void end(boolean interrupted) {
			this.drivebase.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}

        @Override
        public void initSendable(SendableBuilder b) {
            super.initSendable(b);
            b.addDoubleProperty("Left Target M/S", this.left, null);
            b.addDoubleProperty("Right Target M/S", this.right, null);
        }


    }
    public static class TankDriveVelocity_P extends CommandBase {

        private final DriveBase drivebase;
        private final DoubleSupplier left, right;
        private final ProfiledPIDController left_fb, right_fb;

        public TankDriveVelocity_P(
            DriveBase db, DoubleSupplier l, DoubleSupplier r
        ) {
            this.drivebase = db;
            this.left = l;
            this.right = r;
            this.left_fb = db.parameters.getProfiledFeedbackController();
            this.right_fb = db.parameters.getProfiledFeedbackController();
        }

        @Override
        public void initialize() {
            this.left_fb.reset(0);
            this.right_fb.reset(0);
        }
        @Override
        public void execute() {
			double
				lt = this.left.getAsDouble(),	// the target velocity from the left input --> METERS PER SECOND
				rt = this.right.getAsDouble(),	// ^^^ for the right side
				lc = this.drivebase.getLeftVelocity(),  // the actual velocity
				rc = this.drivebase.getRightVelocity();
            this.drivebase.setDriveVoltage(
				this.drivebase.feedforward.calculate(lt) +  // the calculated feedforward
					this.left_fb.calculate(lc, lt),   		// add the feedback adjustment
				this.drivebase.feedforward.calculate(rt) +
					this.right_fb.calculate(rc, rt)
			);
        }
        @Override
		public void end(boolean interrupted) {
			this.drivebase.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}


    }


    public static class ActivePark extends CommandBase {

        private final DriveBase drivebase;
        private final double volts_per_meter;
        private double linit, rinit;

        public ActivePark(DriveBase db, double p) { // p is the proportional gain, in volts per meter [error]
            this.drivebase = db;
            this.volts_per_meter = p;
        }

        @Override
        public void initialize() {
            this.linit = this.drivebase.getLeftPosition();
            this.rinit = this.drivebase.getRightPosition();
        }
        @Override
        public void execute() {
            double le = this.drivebase.getLeftPosition() - this.linit;
            double re = this.drivebase.getRightPosition() - this.rinit;
            this.drivebase.setDriveVoltage(
                -le * this.volts_per_meter,     // we are assuming that positive position for the encoders is the same direction as positive voltage
                -re * this.volts_per_meter
            );
        }
        @Override
        public void end(boolean interrupted) {
            this.drivebase.setDriveVoltage(0.0, 0.0);
        }
        @Override
        public boolean isFinished() {
            return false;
        }


    }


}