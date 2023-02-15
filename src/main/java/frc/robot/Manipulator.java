package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.*;


public final class Manipulator {
    
    public static final class Arm implements Subsystem {

        private final WPI_TalonSRX motor;

        public Arm(WPI_TalonSRX m) {
            this.motor = m;

            this.motor.configFactoryDefault();
            this.motor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
            this.motor.setSelectedSensorPosition(0.0);      // or other relevant value here
            // invert?
        }

    }
    public static final class Grabber implements Subsystem {

        private final MotorController main;
        private final Servo wrist;
        private final Encoder position;

        public Grabber(MotorController m, Servo w, Encoder p) {
            this.main = m;
            this.wrist = w;
            this.position = p;
        }

    }

    private final Arm arm;
    private final Grabber grabber;

    public Manipulator(Arm a, Grabber g) {
        this.arm = a;
        this.grabber = g;
    }



    public static class OperateClaw extends CommandBase {

        private final Grabber grabber;
        private final BooleanSupplier opening, closing;
        private final double volts_output;

        public OperateClaw(Grabber g, BooleanSupplier o, BooleanSupplier c, double volts_out) {
            this.grabber = g;
            this.opening = o;
            this.closing = c;
            this.volts_output = volts_out;
        }

        @Override
        public void initialize() {

        }
        @Override
        public void execute() {
            if(this.opening.getAsBoolean()) {   // and not opened too wide
                // open
            } else if(this.closing.getAsBoolean()) {    // and not already completely closed
                // close
            } else {
                // stop voltage
            }
        }
        @Override
        public boolean isFinished() {
            return false;
        }
        @Override
        public void end(boolean isfinished) {
            // stop 
        }

    }


}
