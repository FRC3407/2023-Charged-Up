package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;


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
		TOP			(2, "Top Camera")/*,
		PIXY2		(3, "PixyCam")*/;

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
		protected CameraSelect selected;

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
			// System.out.println("Initialized Camera Control Command.");
			CameraSelect[] vals = CameraSelect.values();
			int id = getSelectedCamera();
			selected = vals[id >= vals.length ? vals.length - 1 : id];
		}
		
		@Override
		public void execute() {
			if(this.increment.getAsBoolean()) {
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
		public boolean runsWhenDisabled() { return true; }
		@Override
		public void end(boolean i) {}



		public static class DirectSwitching extends CameraControl {

			protected final BooleanSupplier
				sel_forward,
				sel_arm,
				sel_top;

			public DirectSwitching(
				BooleanSupplier inc, BooleanSupplier dec, BooleanSupplier vrb,
				BooleanSupplier fwd, BooleanSupplier arm, BooleanSupplier top
			) {
				super(inc, dec, vrb);
				this.sel_forward = fwd;
				this.sel_arm = arm;
				this.sel_top = top;
			}
			public DirectSwitching(
				BooleanSupplier inc, BooleanSupplier vrb,
				BooleanSupplier fwd, BooleanSupplier arm, BooleanSupplier top
			) { this(inc, ()->false, vrb, fwd, arm, top); }
			public DirectSwitching(
				BooleanSupplier inc,
				BooleanSupplier fwd, BooleanSupplier arm, BooleanSupplier top
			) { this(inc, ()->false, ()->false, fwd, arm, top); }

			@Override
			public void execute() {
				super.execute();
				if(this.sel_forward.getAsBoolean()) { Vision.selectCamera(super.selected = CameraSelect.FORWARD); }
				if(this.sel_arm.getAsBoolean()) { Vision.selectCamera(super.selected = CameraSelect.ARM); }
				if(this.sel_top.getAsBoolean()) { Vision.selectCamera(super.selected = CameraSelect.TOP); }
			}

		}

	}










	public static class photonVision {
		String camera_name = "cameraZ";

		// Creates a photon camera w/ same name as camera
		PhotonCamera camera = new PhotonCamera(camera_name);


		public PhotonPipelineResult getLResult()
		{
			// Query the latest result from PhotonVision (huh? -z)
			// Info about currently detected targets
			// returns a container (query? -z) of information w/ timestamp
			PhotonPipelineResult result = camera.getLatestResult();
			return result;

			// should "result" be an instance variable?
			// should camera be a parameter that gets passed in?
		}

		public boolean hasTargets()
		{
			// This checks if the latest result(!!! -z) has any targets.
			boolean hasTargets = getLResult().hasTargets();
			return hasTargets;

			// should result be a parameter that gets passed in?
		}

		/* 
			What is a PhotonTrackedTarget???
			A tracked target contains information about each target from a pipeline result. 
			Information includes:
				yaw, 
				pitch, 
				area, 
				and robot relative pose.

			// results are pipelined in?
		*/ 
		
		public List<PhotonTrackedTarget> getTargets() {
			// Get a list of currently tracked targets.
			List<PhotonTrackedTarget> targets = getLResult().getTargets();
			return targets;
		}

		public PhotonTrackedTarget bestTarget()
		{
			// Get the current best target.
			PhotonTrackedTarget target = getLResult().getBestTarget();
			return target;
		}

		// what is this?
		public double getYaw(PhotonTrackedTarget target)
		{
			return target.getYaw();
		}

		// what is this?
		public double getPitch(PhotonTrackedTarget target)
		{
			return target.getPitch();
		}

		public double getArea(PhotonTrackedTarget target)
		{
			return target.getArea();
		}

		public double getSkew(PhotonTrackedTarget target)
		{
			return target.getSkew();
		}

		// look at docs for Transform2d 
		// https://docs.wpilib.org/en/latest/docs/software/advanced-controls/geometry/transformations.html#transform2d-and-twist2d
		public Transform2d getPose(PhotonTrackedTarget target)
		{
			Transform3d t3d = target.getBestCameraToTarget();
			Transform2d pose = new Transform2d(t3d.getTranslation().toTranslation2d(), t3d.getRotation().toRotation2d());
			return pose;
		}
		
		public List<TargetCorner> getCorners(PhotonTrackedTarget target)
		{
			List<TargetCorner> corners = target.getDetectedCorners();
			return corners;
		}


		// Getting April Tag Data:
		public int getID(PhotonTrackedTarget target)
		{
			// The ID of the detected fiducial marker.
			int targetID = target.getFiducialId();
			return targetID;
		}

		public double poseAmbig(PhotonTrackedTarget target)
		{
			double poseAmbiguity = target.getPoseAmbiguity();
			return poseAmbiguity;
		}
		
		public Transform3d bestCamera2Target(PhotonTrackedTarget target)
		{
			Transform3d bestCameraToTarget = target.getBestCameraToTarget();
			return bestCameraToTarget;
		}
		
		public Transform3d altCamera2Target(PhotonTrackedTarget target)
		{
			Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
			return alternateCameraToTarget;
		}
		
		// Say click, Take a pic
		// https://www.youtube.com/watch?v=gktOri8vP7E&t=9s
		public void inputSnap()
		{
			// Capture pre-process camera stream image
			camera.takeInputSnapshot();
		}

		public void outputSnap()
		{
			// Capture post-process camera stream image
			camera.takeOutputSnapshot();
		}

	}


}
