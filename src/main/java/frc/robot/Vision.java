package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedFloatArray;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


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

			this.all_estimations = this.vbase.getDoubleArrayTopic("Pose Estimations/Individual").subscribe(
				new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.combined_estimations = this.vbase.getDoubleArrayTopic("Pose Estimations/Combined").subscribe(
				new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.estimate_errors = this.vbase.getFloatArrayTopic("Pose Estimations/RMSE").subscribe(
				new float[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.tag_distances = this.vbase.getFloatArrayTopic("Pose Estimations/Tag Distances").subscribe(
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
			all_estimations,
			combined_estimations,
			retrorefl_centers;
		public final FloatArraySubscriber
			estimate_errors,
			tag_distances;
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
		double[] d = nt.all_estimations.get();
		return extract3dPoses(d, p);
	}
	public static Pose3d[] getCurrentEstimations() {
		return getCurrentEstimations(null);
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

		public static final BoundingVolume
			FIELD_BOUNDS_3D = new BoundingVolume(
				0.0, Constants.Field.FIELD_LENGTH,
				0.0, Constants.Field.FIELD_WIDTH,
				0.0, 3.0);

		public static final Transform3d
			FORWARD_C2R = new Transform3d(
				new Translation3d(),
				new Rotation3d()),
			ARM_C2R = new Transform3d(
				new Translation3d(),
				new Rotation3d()),
			TOP_C2R = new Transform3d(
				new Translation3d(
					Units.inchesToMeters(3),	// forward amount
					Units.inchesToMeters(-7),	// rightward amount
					Units.inchesToMeters(30)),	// upward amount
				new Rotation3d(0.0, -20, 0.0));	// angled slightly downward but otherwise facing forward
		public static final Transform3d[]
			CAMERA_TO_ROBOT_POSES = new Transform3d[]{ FORWARD_C2R, ARM_C2R, TOP_C2R };

		public static Transform3d getCameraTranslation(CameraSelect c) {
			return CAMERA_TO_ROBOT_POSES[c.id];
		}

		public static final double getXYDeviation(double dx) {
			if(dx < 1) { return 0.005; }
			if(dx < 2) { return 0.02 * dx; }	// from log data: deviation follows ~ 0.103x-0.157 (approximated here), but we also don't want negative values
			return 0.1 * dx - 0.15;
		}
		public static final double getThetaDeviation(double dx) {
			return 0.01 * dx + 0.025;			// this is more of a guesstimate
		}



		public static final class RawEstimationBuffer {
			public TimestampedDoubleArray[] estimations;
			public TimestampedFloatArray[] errors, distances;
		}
		public static final class Estimation {
			public Pose3d pose;
			public float rmse, distance;
		}
		public static final class EstimationFrame {
			public long tstamp;
			public Estimation[] estimations;
		}

		public static RawEstimationBuffer readEstimationQueue(RawEstimationBuffer buff) {
			if(buff == null) {
				buff = new RawEstimationBuffer();
			}
			buff.estimations = Vision.nt.all_estimations.readQueue();
			buff.errors = Vision.nt.estimate_errors.readQueue();
			buff.distances = Vision.nt.tag_distances.readQueue();
			return buff;
		}
		public static EstimationFrame[] extractEstimations(RawEstimationBuffer buff, EstimationFrame[] frames) {
			if(buff != null && buff.estimations != null) {
				final int len = buff.estimations.length;
				if(frames == null || frames.length != len) {
					frames = new EstimationFrame[len];
				}
				for(int i = 0; i < len; i++) {
					frames[i] = new EstimationFrame();
					TimestampedDoubleArray posedata = buff.estimations[i];
					frames[i].tstamp = posedata.timestamp;
					Pose3d[] poses = Vision.extract3dPoses(posedata.value, null);
					frames[i].estimations = new Estimation[poses.length];
					for(int p = 0; p < poses.length; p++) {			// maybe assert that all the inner data lengths are the same
						frames[i].estimations[p].pose = poses[p];
						frames[i].estimations[p].rmse = buff.errors[i].value[p];		// or check during each loop
						frames[i].estimations[p].distance = buff.distances[i].value[p];
					}
				}
			}
			return frames;
		}
		public static boolean testFieldBounds(Estimation e, BoundingVolume v) {
			return insideBuffered(e.pose, v, getXYDeviation(e.distance));
		}
		public static Estimation[] sortClosestNxN(Estimation[] e, int n, int s) {
			Estimation[] ret = new Estimation[n];
			for(int x = 0; x < n-1; x++) {
				double min = Double.POSITIVE_INFINITY;
				for(int b = (s*(x + 1)); b < (s*(x + 2)); b++) {
					if(x > 0) {
						double d = ret[x].pose.getTranslation().getDistance(e[b].pose.getTranslation());
						if(d < min) {
							ret[x + 1] = e[b];
							min = d;
						}
					} else {
						for(int a = s*x; a < (s*(x + 1)); a++) {
							double d = e[a].pose.getTranslation().getDistance(e[b].pose.getTranslation());
							if(d < min) {
								ret[x] = e[a];
								ret[x + 1] = e[b];
								min = d;
							}
						}
					}
				}
			}
			return ret;
		}

		public static class BoundingVolume {
			public double[] _a, _b;
			public BoundingVolume() {
				this._a = new double[3];
				this._b = new double[3];
			}
			public BoundingVolume(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
				this._b[0] = xmin;
				this._a[0] = xmax;
				this._a[1] = ymax;
				this._b[1] = ymin;
				this._a[2] = zmax;
				this._b[2] = zmin;
			}
			public BoundingVolume(double[] a, double[] b) {
				if(a.length == 3) { this._a = a.clone(); } else { this._a = new double[3]; }
				if(b.length == 3) { this._b = b.clone(); } else { this._b = new double[3]; }
			}
		}
		

		public static boolean inside(Pose3d p, BoundingVolume v) {
			boolean
				x = p.getX() > v._b[0] && p.getX() < v._a[0],
				y = p.getY() > v._b[1] && p.getY() < v._a[1],
				z = p.getZ() > v._b[2] && p.getZ() < v._a[2];
			return x && y && z;
		}
		public static boolean insideBuffered(Pose3d p, BoundingVolume v, double b) {
			boolean
				x = (p.getX() > v._b[0] - b) && (p.getX() < v._a[0] + b),
				y = (p.getY() > v._b[1] - b) && (p.getY() < v._a[1] + b),
				z = (p.getZ() > v._b[2] - b) && (p.getZ() < v._a[2] + b);
			return x && y && z;
		}
		public static boolean outside(Pose3d p, BoundingVolume v) {
			boolean
				x = p.getX() < v._b[0] || p.getX() > v._a[0],
				y = p.getY() < v._b[1] || p.getY() > v._a[1],
				z = p.getZ() < v._b[2] || p.getZ() > v._a[2];
			return x && y && z;
		}
		public static boolean outsideBuffered(Pose3d p, BoundingVolume v, double b) {
			boolean
				x = (p.getX() < v._b[0] + b) || (p.getX() > v._a[0] - b),
				y = (p.getY() < v._b[1] + b) || (p.getY() > v._a[1] - b),
				z = (p.getZ() < v._b[2] + b) || (p.getZ() > v._a[2] - b);
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
		public static boolean outside(Pose3d p, BoundingVolume... vs) {
			for(BoundingVolume v : vs) {
				if(!outside(p, v)) {
					return false;
				}
			}
			return true;
		}




		public static void fuseVision(DriveBase db) {
			final int c_id = Vision.getSelectedCamera();	// or equivelant timestamp-sampled history of camera selection
			RawEstimationBuffer rbuff = PoseEstimation.readEstimationQueue(null);
			EstimationFrame[] frames = PoseEstimation.extractEstimations(rbuff, null);
			for(EstimationFrame frame : frames) {
				if(frame.estimations.length > 2) {	// need to sort through multiple tag detections
					
					Estimation[] closest = PoseEstimation.sortClosestNxN(frame.estimations, frame.estimations.length / 2, 2);

				} else {	// only one estimation to deal with
					Estimation best = null;
					switch(frame.estimations.length) {
						case 0:
						default:
							continue;
						case 1:
						{
							best = frame.estimations[0];
							break;
						}
						case 2:
						{
							if(frame.estimations[0].rmse < frame.estimations[1].rmse * 0.15) {
								best = frame.estimations[0];
							} else if(frame.estimations[1].rmse < frame.estimations[0].rmse * 0.15) {
								best = frame.estimations[1];
							} else {
								// compare to current pose, rotation, etc..?
							}
							break;
						}
					}
					// translate to robot pose
					double dvxy = getXYDeviation(best.distance);
					if(insideBuffered(best.pose, FIELD_BOUNDS_3D, dvxy)) {
						db.applyVisionUpdate(
							best.pose.toPose2d(),
							frame.tstamp / 1e6,
							dvxy, getThetaDeviation(best.distance)
						);
					}
				}
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
