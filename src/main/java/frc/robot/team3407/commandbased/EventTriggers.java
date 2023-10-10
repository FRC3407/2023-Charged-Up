package frc.robot.team3407.commandbased;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;


/**
 * Triggers that become active during specific portions of a game. 
 */
public final class EventTriggers {

	public static class EnabledTrigger extends Trigger {
		private static EnabledTrigger singleton = new EnabledTrigger();
		private EnabledTrigger() {
			super(DriverStation::isEnabled);
		}
		public static EnabledTrigger Get() { return EnabledTrigger.singleton; }
		public static Trigger OnTrue(Command c) { return Get().onTrue(c); }
		public static Trigger OnFalse(Command c) { return Get().onFalse(c); }
		public static Trigger WhileTrue(Command c) { return Get().whileTrue(c); }
		public static Trigger WhileFalse(Command c) { return Get().whileFalse(c); }
		public static Trigger ToggleTrue(Command c) { return Get().toggleOnTrue(c); }
		public static Trigger ToggleFalse(Command c) { return Get().toggleOnFalse(c); }

		public static Trigger makeWithLoop(EventLoop l) { return new Trigger(l, DriverStation::isEnabled); }
	}
	
	public static class DisabledTrigger extends Trigger {
		private static DisabledTrigger singleton = new DisabledTrigger();
		private DisabledTrigger() {
			super(DriverStation::isDisabled);
		}
		public static DisabledTrigger Get() { return DisabledTrigger.singleton; }
		public static Trigger OnTrue(Command c) { return Get().onTrue(c); }
		public static Trigger OnFalse(Command c) { return Get().onFalse(c); }
		public static Trigger WhileTrue(Command c) { return Get().whileTrue(c); }
		public static Trigger WhileFalse(Command c) { return Get().whileFalse(c); }
		public static Trigger ToggleTrue(Command c) { return Get().toggleOnTrue(c); }
		public static Trigger ToggleFalse(Command c) { return Get().toggleOnFalse(c); }

		public static Trigger makeWithLoop(EventLoop l) { return new Trigger(l, DriverStation::isDisabled); }
	}

	public static class TeleopTrigger extends Trigger {
		private static TeleopTrigger singleton = new TeleopTrigger();
		private TeleopTrigger() {
			super(DriverStation::isTeleopEnabled);
		}
		public static TeleopTrigger Get() { return TeleopTrigger.singleton; }
		public static Trigger OnTrue(Command c) { return Get().onTrue(c); }
		public static Trigger OnFalse(Command c) { return Get().onFalse(c); }
		public static Trigger WhileTrue(Command c) { return Get().whileTrue(c); }
		public static Trigger WhileFalse(Command c) { return Get().whileFalse(c); }
		public static Trigger ToggleTrue(Command c) { return Get().toggleOnTrue(c); }
		public static Trigger ToggleFalse(Command c) { return Get().toggleOnFalse(c); }

		public static Trigger makeWithLoop(EventLoop l) { return new Trigger(l, DriverStation::isTeleopEnabled); }
	}

	public static class AutonomousTrigger extends Trigger {
		private static AutonomousTrigger singleton = new AutonomousTrigger();
		private AutonomousTrigger() {
			super(DriverStation::isAutonomousEnabled);
		}
		public static AutonomousTrigger Get() { return AutonomousTrigger.singleton; }
		public static Trigger OnTrue(Command c) { return Get().onTrue(c); }
		public static Trigger OnFalse(Command c) { return Get().onFalse(c); }
		public static Trigger WhileTrue(Command c) { return Get().whileTrue(c); }
		public static Trigger WhileFalse(Command c) { return Get().whileFalse(c); }
		public static Trigger ToggleTrue(Command c) { return Get().toggleOnTrue(c); }
		public static Trigger ToggleFalse(Command c) { return Get().toggleOnFalse(c); }

		public static Trigger makeWithLoop(EventLoop l) { return new Trigger(l, DriverStation::isAutonomousEnabled); }
	}

	public static class TestTrigger extends Trigger {
		private static TestTrigger singleton = new TestTrigger();
		private TestTrigger() {
			super(DriverStation::isTest);
		}
		public static TestTrigger Get() { return TestTrigger.singleton; }
		public static Trigger OnTrue(Command c) { return Get().onTrue(c); }
		public static Trigger OnFalse(Command c) { return Get().onFalse(c); }
		public static Trigger WhileTrue(Command c) { return Get().whileTrue(c); }
		public static Trigger WhileFalse(Command c) { return Get().whileFalse(c); }
		public static Trigger ToggleTrue(Command c) { return Get().toggleOnTrue(c); }
		public static Trigger ToggleFalse(Command c) { return Get().toggleOnFalse(c); }

		public static Trigger makeWithLoop(EventLoop l) { return new Trigger(l, DriverStation::isTest); }
	}

}