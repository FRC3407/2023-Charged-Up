package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Vision {

	public static PubSubOption NT_ENTRY_DEFAULT = PubSubOption.periodic(0.02);

	public static class NT {
		private NT() {
			this.vbase = NetworkTableInstance.getDefault().getTable("Vision");
			this.avail_cams = this.vbase.getIntegerTopic("Available Outputs").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.active_cam = this.vbase.getIntegerTopic("Active Camera Thread").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.overlay_verbosity = this.vbase.getIntegerTopic("Overlay Verbosity").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
		}
		private static NT global;
		public static NT get() {
			if(global == null) {
				global = new NT();
			}
			return global;
		}

		public final NetworkTable vbase;
		public final IntegerEntry avail_cams;
		public final IntegerEntry active_cam;
		public final IntegerEntry overlay_verbosity;
	}

	public static NT nt = null;

	static void init() {
		nt = NT.get();
	}

	public static enum CameraSelect {
		FORWARD		(0, "Forward Facing Camera"),
		ARM			(1, "Arm Camera"),
		TOP			(2, "Top Camera"),
		PIXY2		(3, "PixyCam");

		public final int id;
		public final String name;
		private CameraSelect(int id, String n) {
			this.id = id;
			this.name = n;
		}

		public void set() { selectCamera(this); }
		public CameraSelect increment() {
			CameraSelect[] vals = CameraSelect.values();
			int idx = this.id + 1 >= vals.length ? 0 : this.id + 1;
			return vals[idx];
		}
		public CameraSelect decrement() {
			CameraSelect[] vals = CameraSelect.values();
			int idx = this.id - 1 < 0 ? vals.length - 1 : this.id - 1;
			return vals[idx];
		}
	}
	public static enum Verbosity {
		NONE	(0),
		SIMPLE	(1),
		COMPLEX	(2);

		public final int val;
		private Verbosity(int v) { this.val = v; }

		public void set() { setVerbosity(this); }
	}


	public static void selectCamera(CameraSelect c) {
		nt.active_cam.set(c.id);
	}
	public static void selectCamera(int c) {
		nt.active_cam.set(c);
	}
	public static int getSelectedCamera() {
		return (int)nt.active_cam.get();
	}
	public static void setVerbosity(Verbosity v) {
		nt.overlay_verbosity.set(v.val);
	}
	public static int getVerbosity() {
		return (int)nt.overlay_verbosity.get();
	}


	public static class CameraControl extends CommandBase {

		private final BooleanSupplier
			increment,
			decrement,
			verbosity;
		private CameraSelect selected;

		public CameraControl(BooleanSupplier inc, BooleanSupplier dec, BooleanSupplier vrb) {
			this.increment = inc;
			this.decrement = dec;
			this.verbosity = vrb;
			super.addRequirements();
		}
		public CameraControl(BooleanSupplier inc, BooleanSupplier vrb) { this(inc, ()->false, vrb); }
		public CameraControl(BooleanSupplier inc) { this(inc, ()->false, ()->false); }

		@Override
		public void initialize() {
			System.out.println("Initialized Camera Control Command.");
			CameraSelect[] vals = CameraSelect.values();
			int id = getSelectedCamera();
			selected = vals[id >= vals.length ? vals.length - 1 : id];
		}
		@Override
		public void execute() {
			if(this.increment.getAsBoolean()) {
				System.out.println("Increment Camera.");
				Vision.selectCamera(this.selected = this.selected.increment());
			} else if(this.decrement.getAsBoolean()) {
				Vision.selectCamera(this.selected = this.selected.decrement());
			}
			if(this.verbosity.getAsBoolean()) {
				Vision.setVerbosity(Vision.getVerbosity() > 0 ? Verbosity.NONE : Verbosity.SIMPLE);
			}
		}
		@Override
		public boolean isFinished() { return false; }
		@Override
		public void end(boolean i) {}

	}

}
