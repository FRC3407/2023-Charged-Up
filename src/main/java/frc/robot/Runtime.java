package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.team3407.Input;
import frc.robot.team3407.Input.InputDevice;
import frc.robot.team3407.commandbased.EventTriggers.*;
import frc.robot.team3407.commandbased.LambdaCommand;
import frc.robot.team3407.debug.Debug;
import frc.robot.team3407.ADIS16470;


public class Runtime extends TimedRobot {

	public InputDevice input = new InputDevice(0);

	public ADIS16470 imu_gyro = new ADIS16470();
	public DriveBase drivebase = new DriveBase(
		Constants.DRIVEBASE_LAYOUT,
		imu_gyro,
		Constants.DRIVEBASE_PARAMS
	);

	public Runtime() {
		System.out.println("ROBOT STARTING.");
		if(this.input.isConnected()) {
			System.out.println("Xbox Controller Connected.");
		} else {
			System.out.println("Checking for inputs...");
			new Thread(()->{
				for(;;) {
					try{ Thread.sleep(500); }	// half a second
					catch(InterruptedException e) { System.out.println(e.getMessage()); }
					if(this.input.isConnected()) {
						System.out.println("Xbox Controller Connected!");
						this.input.getTrigger(Input.Xbox.Digital.A.value).onTrue(new LambdaCommand(()->System.out.println("Hello World")));
						
						TeleopTrigger.Get().onTrue(
							new DriveBase.TankDriveVelocity(
								this.drivebase,
								Input.Xbox.Analog.LY.getSupplier(this.input),
								Input.Xbox.Analog.RY.getSupplier(this.input)
							)
						);
						
						return;
					}
				}
			}).start();
		}
	}

	@Override
	public void robotInit() {}
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

}