package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;


public final class Vision {

	public static PubSubOption NT_ENTRY_DEFAULT = PubSubOption.periodic(0.02);
	public static PubSubOption[] NT_DATA_SUBSCRIBER = new PubSubOption[]{
		NT_ENTRY_DEFAULT, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10) };

	public static class NT {
		private NT() {
			this.vbase = NetworkTableInstance.getDefault().getTable("Vision");
			this.avail_cams = this.vbase.getIntegerTopic("Available Outputs").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.active_cam = this.vbase.getIntegerTopic("Active Camera Thread").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.overlay_verbosity = this.vbase.getIntegerTopic("Overlay Verbosity").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.cam_exposure = this.vbase.getIntegerTopic("Camera Exposure").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.downscaling = this.vbase.getIntegerTopic("Output Downscale").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.aprilp_mode = this.vbase.getIntegerTopic("AprilTag Mode").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);
			this.retror_mode = this.vbase.getIntegerTopic("RetroRefl Mode").getEntry(
				0, Vision.NT_ENTRY_DEFAULT);

			this.active_estimations = this.vbase.getDoubleArrayTopic("Pose Estimations/Active").subscribe(
				new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.extra_estimations = this.vbase.getDoubleArrayTopic("Pose Estimations/Extra").subscribe(
				new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.estimate_errors = this.vbase.getFloatArrayTopic("Pose Estimations/RMSE").subscribe(
				new float[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.retrorefl_centers = this.vbase.getDoubleArrayTopic("RetroReflective Detections/Centers").subscribe(
				new double[]{}, Vision.NT_DATA_SUBSCRIBER);
		}
		private static NT global;
		public static NT get() {
			if(global == null) {
				global = new NT();
			}
			return global;
		}

		public final NetworkTable vbase;
		public final IntegerEntry
			avail_cams,
			active_cam,
			overlay_verbosity,
			cam_exposure,
			downscaling,
			aprilp_mode,
			retror_mode;
		public final DoubleArraySubscriber
			active_estimations,
			extra_estimations,
			retrorefl_centers;
		public final FloatArraySubscriber
			estimate_errors;
	}

	public static NT nt = null;

	static void init() {
		nt = NT.get();
	}

	public static enum CameraSelect {
		FORWARD		(0, "Forward Facing Camera"),
		ARM			(1, "Arm Camera"),
		TOP			(2, "Top Camera");

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

	public static Pose3d[] extract3dPoses(double[] raw, Pose3d[] buff) {
		int len = raw.length / 7;
		if(buff == null || buff.length != len) {
			buff = new Pose3d[len];
		}
		for(int i = 0; i / 7 < buff.length;) {
			buff[i / 7] = new Pose3d(
				new Translation3d(
					raw[i++],
					raw[i++],
					raw[i++]
				),
				new Rotation3d(
					new Quaternion(
						raw[i++],
						raw[i++],
						raw[i++],
						raw[i++]
					))
			);
		}
		return buff;
	}

	public static Pose3d[] getCurrentEstimations(Pose3d[] p) {
		double[] d = nt.active_estimations.get();
		return extract3dPoses(d, p);
	}
	public static Pose3d[] getCurrentEstimations() {
		return getCurrentEstimations(null);
	}
	public static Pose3d[][] getQueuedEstimations() {
		TimestampedDoubleArray[] d = nt.active_estimations.readQueue();
		if(d.length > 0) {
			Pose3d[][] arr = new Pose3d[d.length][];
			for(int i = 0; i < arr.length; i++) {
				arr[i] = extract3dPoses(d[i].value, null);
			}
			return arr;
		}
		return null;
	}
	public static TimestampedDoubleArray getNewestEstimation() {
		TimestampedDoubleArray[] d = nt.active_estimations.readQueue();
		if(d.length > 0) {
			return d[d.length - 1];
		}
		return null;
	}


	public static class PoseUpdater extends CommandBase {

		private final FieldObject2d obj;
		private ArrayList<Pose2d> poses = new ArrayList<>();
		private Pose3d[] raw = null;

		public PoseUpdater(FieldObject2d o) {
			this.obj = o;
		}

		@Override
		public void initialize() {
			this.obj.setPose(new Pose2d(1, 1, new Rotation2d(1)));
		}
		@Override
		public void execute() {
			this.raw = Vision.getCurrentEstimations(this.raw);
			if(this.raw.length > 0) {
				this.poses.clear();
				for(int i = 0; i < raw.length; i++) {
					this.poses.add(this.raw[i].toPose2d());
				}
				this.obj.setPoses(this.poses);
			}
		}
		@Override
		public boolean runsWhenDisabled() { return true; }

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









	public static final class PoseEstimation {

		public static class BoundingVolume {
			double[] xyz_a = new double[3], xyz_b = new double[3];
		}
		public static BoundingVolume createMinMax(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
			BoundingVolume v = new BoundingVolume();
			v.xyz_a[0] = xmax;
			v.xyz_b[0] = xmin;
			v.xyz_a[1] = ymax;
			v.xyz_b[1] = ymin;
			v.xyz_a[2] = zmax;
			v.xyz_b[2] = zmin;
			return v;
		}

		public static final double
			FIELD_WIDTH_METERS = 8.0137,
			FIELD_LENGTH_METERS = 16.54175,
			MAX_HEIGHT = 3.0;
		

		public static boolean inside(Pose3d p, BoundingVolume v) {
			boolean
				x = p.getX() > v.xyz_b[0] && p.getX() < v.xyz_a[0],
				y = p.getY() > v.xyz_b[1] && p.getY() < v.xyz_a[1],
				z = p.getZ() > v.xyz_b[2] && p.getZ() < v.xyz_a[2];
			return x && y && z;
		}
		public static boolean inside(Pose3d p, BoundingVolume... vs) {
			for(BoundingVolume v : vs) {
				if(!inside(p, v)) {
					return false;
				}
			}
			return true;
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
