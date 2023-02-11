package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.team3407.drive.Types.DriveMap_4;
import frc.robot.team3407.drive.Types.Inversions;


public class DriveBase implements Subsystem, Sendable { 
    //what is this? why not "public class ClosedLoopDifferentialDrive extends DriveBase"
    //RIP the more I think about this the less I understand 
    
    public static class ClosedLoopParams {
        public final Inversions
            encoder_inversions;
        public final double
            track_width_meters,
            wheel_diameter_meters,
            volt_seconds_per_meter, 		// "kV" (volts * seconds / meters) -> "voltage to hold a velocity"
		    volt_seconds_sqrd_per_meter,	// "kA" (volts * seconds^2 / meters) -> "voltage to hold an acceleration"
		    volt_seconds_per_meter_gain
        ;
        public ClosedLoopParams(
            double trackwidth,
            double wheeldiameter,
            Inversions encoderinversions
        ) 
        {
            this.track_width_meters = trackwidth;
            this.wheel_diameter_meters = wheeldiameter;
            this.encoder_inversions = encoderinversions;
        }
    }
    


    private Gyro 
        gyro;
    private WPI_TalonSRX
        left, left2,
        right, right2;
    private ClosedLoopParams
        parameters;
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final SimpleMotorFeedforward feedforward;


    public DriveBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, ClosedLoopParams params) {
        this.parameters = params;
        this.gyro = gy;
        this.left = map.front_left;
        this.right = map.front_right;
        this.left2 = map.back_left;
        this.right2 = map.back_right;
        this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d(), 0, 0); 
        this.kinematics = new DifferentialDriveKinematics(getContinuousAngle());
        this.feedforward = new SimpleMotorFeedforward(getHeading(), getContinuousAngle());

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
        // update field position and other tethered stats
    }
    @Override
    public void initSendable(SendableBuilder b) {
        // add data to send
    }

    public void setDriveVoltage(double lv, double rv) {
        this.left.setVoltage(lv);
        this.right.setVoltage(rv);
    }
    public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
    public void zeroHeading() {
		this.gyro.reset();
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

    public Pose2d getCurrentPose() {	// in meters
		return this.odometry.getPoseMeters();
	}
    // this was in Rapid React so I added it Justin Case:
    // public Pose2d getTotalPose() {	// in meters
	// 	//return this.odometry.getPoseMeters().plus(this.position_offset);
	// 	Pose2d current = this.getCurrentPose();
	// 	return this.position_offset.plus(new Transform2d(current.getTranslation(), current.getRotation()));
	// }
    
    
    public FollowTrajectory followTrajectory(Trajectory t)
    {
        return new FollowTrajectory(this, t);
    }

    public static class FollowTrajectory extends CommandBase{
        private final DriveBase drivebase; 
        private final Trajectory trajectory;
        private final RamseteCommand controller;
        private final boolean stop; // for what?

        @Override public void initialize(){}
        @Override public void execute(){}
        @Override public void end(boolean interrupted){}
        @Override public boolean isFinished(){
            return false; // just so it isn't red anymore 
        }


        FollowTrajectory(DriveBase db, Trajectory t)
        {
            this(db, t, true);
        }
        // FollowTrajectory(DriveBase db, Path json_path)
        // {
        //     this(db, json_path, true);
        //     // calciumatator
        // }
        FollowTrajectory(DriveBase db, Trajectory t, boolean s)
        {
            super();
            this.trajectory = t;
            this.stop = s;
            this.drivebase = db;
            this.controller = new RamseteCommand
                (
                this.trajectory, 
                this.drivebase::getCurrentPose, 
                /* recives a function, and keeps getting new info */
                new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
                // add the things to constants
                /* why all this? Cant we initialize it when it's declared? */
                this.drivebase.feedforward, 
                this.drivebase.kinematics,  
                this.drivebase::getWheelSpeeds, 
                new PIDController(this.drivebase.params.kP(), 0, 0), //huh? 
                // Closed look params look for getFeedbackController 
                // Ask Sam to explain it l8tr 
                new PIDController(this.drivebase.params.kP(), 0, 0), 
                this.drivebase::setDriveVoltage
                );
        }

        // FollowTrajectory(DriveBase db, Path json_path, boolean s)
        // {
        //     // this isn't right
        //     super(db);
        //     this.Trajectory = json_path;
        //     this.stop = s;
        // }


    }


}