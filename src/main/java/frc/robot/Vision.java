package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Field;


public final class Vision {

	public static final PubSubOption
		NT_ENTRY_DEFAULT = PubSubOption.periodic(0.02);
	public static final PubSubOption[]
		NT_DATA_SUBSCRIBER = new PubSubOption[]{ NT_ENTRY_DEFAULT,
			PubSubOption.keepDuplicates(true),
			PubSubOption.pollStorage(10)
		};

	public static class NT {
		private NT() {
			this.vbase = NetworkTableInstance.getDefault().getTable("Vision");
			this.avail_cams =			this.vbase.getIntegerTopic("Available Outputs")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.active_cam =			this.vbase.getIntegerTopic("Active Camera Thread")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.overlay_verbosity =	this.vbase.getIntegerTopic("Overlay Verbosity")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.cam_exposure =			this.vbase.getIntegerTopic("Camera Exposure")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.downscaling =			this.vbase.getIntegerTopic("Output Downscale")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.aprilp_mode =			this.vbase.getIntegerTopic("AprilTag Mode")
											.getEntry(0, Vision.NT_DATA_SUBSCRIBER);
			this.retror_mode =			this.vbase.getIntegerTopic("RetroRefl Mode")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);

			this.all_estimations =		this.vbase.getDoubleArrayTopic("Pose Estimations/Individual")
											.subscribe(new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.combined_estimations =	this.vbase.getDoubleArrayTopic("Pose Estimations/Combined")
											.subscribe(new double[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.estimate_errors =		this.vbase.getFloatArrayTopic("Pose Estimations/RMSE")
											.subscribe(new float[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.tag_distances =		this.vbase.getFloatArrayTopic("Pose Estimations/Tag Distances")
											.subscribe(new float[]{}, Vision.NT_DATA_SUBSCRIBER);
			this.retrorefl_centers =	this.vbase.getDoubleArrayTopic("RetroReflective Detections/Centers")
											.subscribe(new double[]{}, Vision.NT_DATA_SUBSCRIBER);
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
	public static void selectCamera(CameraSelect c) {
		nt.active_cam.set(c.id);
	}
	public static void selectCamera(int c) {
		nt.active_cam.set(c);
	}
	public static int getSelectedCamID() {
		return (int)nt.active_cam.get();
	}
	public static CameraSelect selectedCameraFromID(int id) {
		final CameraSelect[] vals = CameraSelect.values();
		return (id < 0 || id >= vals.length) ? null : vals[id];
	}

	public static enum Verbosity {
		NONE	(0),
		SIMPLE	(1),
		COMPLEX	(2);

		public final int val;
		private Verbosity(int v) { this.val = v; }

		public void set() { setVerbosity(this); }
	}
	public static void setVerbosity(Verbosity v) {
		nt.overlay_verbosity.set(v.val);
	}
	public static int getVerbosity() {
		return (int)nt.overlay_verbosity.get();
	}


	public static void setExposure(int ex) {
		nt.cam_exposure.set(ex);
	}
	public static int getExposure() {
		return (int)nt.cam_exposure.get();
	}
	public static void setStreamDownscale(int ds) {
		nt.downscaling.set(ds);
	}
	public static int getStreamDownscale() {
		return (int)nt.downscaling.get();
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
			int id = getSelectedCamID();
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





	public static void setAprilTagMode(int m) {
		nt.aprilp_mode.set(m);
	}
	public static void setAprilTagCamera(CameraSelect c) {
		nt.aprilp_mode.set(c.id);
	}
	public static void setAprilTagAutomatic() {
		nt.aprilp_mode.set(-1);
	}
	public static void setAprilTagDisabled() {
		nt.aprilp_mode.set(-2);
	}
	public static int getAprilTagMode() {
		return (int)nt.aprilp_mode.get();
	}
	public static int aprilTagModeToCamID(int m) {
		if(m > -1) { return -1; }
		else if(m == -1) { return getSelectedCamID(); }
		else { return m; }
	}
	public static int getAprilTagCamID() {
		return aprilTagModeToCamID(getAprilTagMode());
	}
	public static TimestampedInteger[] aprilTagModeHistory() {
		return nt.aprilp_mode.readQueue();
	}

	public static void setRetroRefMode(int m) {
		nt.retror_mode.set(m);
	}
	public static void setRetroRefCamera(CameraSelect c) {
		nt.retror_mode.set(c.id);
	}
	public static void setRetroRefAutomatic() {
		nt.retror_mode.set(-1);
	}
	public static void setRetroRefDisabled() {
		nt.retror_mode.set(-2);
	}
	public static int getRetroRefMode() {
		return (int)nt.retror_mode.get();
	}
	public static int retroRefModeToCamID(int m) {
		if(m > -1) { return -1; }
		else if(m == -1) { return getSelectedCamID(); }
		else { return m; }
	}
	public static int getRetroRefCamID() {
		return retroRefModeToCamID(getRetroRefMode());
		
	}










	public static final class PoseEstimation {

		// public static class BoundingVolume {
		// 	public double[] _a, _b;
		// 	public BoundingVolume() {
		// 		this._a = new double[3];
		// 		this._b = new double[3];
		// 	}
		// 	public BoundingVolume(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) {
		// 		this._a = new double[3];
		// 		this._b = new double[3];
		// 		this._b[0] = xmin;
		// 		this._a[0] = xmax;
		// 		this._a[1] = ymax;
		// 		this._b[1] = ymin;
		// 		this._a[2] = zmax;
		// 		this._b[2] = zmin;
		// 	}
		// 	public BoundingVolume(double[] a, double[] b) {
		// 		if(a.length == 3) { this._a = a.clone(); } else { this._a = new double[3]; }
		// 		if(b.length == 3) { this._b = b.clone(); } else { this._b = new double[3]; }
		// 	}
		// }

		// public static boolean inside(Pose3d p, BoundingVolume v) {
		// 	return (
		// 		(p.getX() > v._b[0]) && (p.getX() < v._a[0]) &&
		// 		(p.getY() > v._b[1]) && (p.getY() < v._a[1]) &&
		// 		(p.getZ() > v._b[2]) && (p.getZ() < v._a[2])
		// 	);
		// }
		// public static boolean insideBuffered(Pose3d p, BoundingVolume v, double b) {
		// 	return (
		// 		(p.getX() > v._b[0] - b) && (p.getX() < v._a[0] + b) &&
		// 		(p.getY() > v._b[1] - b) && (p.getY() < v._a[1] + b) &&
		// 		(p.getZ() > v._b[2] - b) && (p.getZ() < v._a[2] + b)
		// 	);
		// }
		// public static boolean outside(Pose3d p, BoundingVolume v) {
		// 	return (
		// 		(p.getX() < v._b[0]) || (p.getX() > v._a[0]) ||
		// 		(p.getY() < v._b[1]) || (p.getY() > v._a[1]) ||
		// 		(p.getZ() < v._b[2]) || (p.getZ() > v._a[2])
		// 	);
		// }
		// public static boolean outsideBuffered(Pose3d p, BoundingVolume v, double b) {
		// 	return (
		// 		(p.getX() < v._b[0] + b) || (p.getX() > v._a[0] - b) ||
		// 		(p.getY() < v._b[1] + b) || (p.getY() > v._a[1] - b) ||
		// 		(p.getZ() < v._b[2] + b) || (p.getZ() > v._a[2] - b)
		// 	);
		// }
		// public static boolean inside(Pose3d p, BoundingVolume... vs) {
		// 	for(BoundingVolume v : vs) {
		// 		if(!inside(p, v)) {
		// 			return false;
		// 		}
		// 	}
		// 	return true;
		// }
		// public static boolean outside(Pose3d p, BoundingVolume... vs) {
		// 	for(BoundingVolume v : vs) {
		// 		if(!outside(p, v)) {
		// 			return false;
		// 		}
		// 	}
		// 	return true;
		// }




		// public static final BoundingVolume
		// 	FIELD_BOUNDS_3D = new BoundingVolume(
		// 		0.0, Constants.Field.FIELD_LENGTH,
		// 		0.0, Constants.Field.FIELD_WIDTH,
		// 		0.0, 2.0);

		public static final Transform3d		// the inverses of the camera locations in robot coord space -- +x is "forward", +y is right, +z is up
			FORWARD_C2R = new Transform3d(
				new Translation3d(
					Units.inchesToMeters(0.0),	// forward amount
					Units.inchesToMeters(0.0),	// rightward amount
					Units.inchesToMeters(0.0)),	// upward amount
				new Rotation3d(0.0, 0.0, 0.0)
			).inverse(),
			ARM_C2R = new Transform3d(
				new Translation3d(
					Units.inchesToMeters(0.0),	// forward amount
					Units.inchesToMeters(0.0),	// rightward amount
					Units.inchesToMeters(0.0)),	// upward amount
				new Rotation3d(0.0, 0.0, 0.0)
			).inverse(),
			TOP_C2R = new Transform3d(
				new Translation3d(
					Units.inchesToMeters(0.0),	// forward amount
					Units.inchesToMeters(0.0),	// rightward amount
					Units.inchesToMeters(0.0)),	// upward amount
				new Rotation3d(0.0, 0.0, 0.0)
			).inverse();
		public static final Transform3d[]
			CAMERA_TO_ROBOT_POSES = new Transform3d[]{ FORWARD_C2R, ARM_C2R, TOP_C2R };

		public static Transform3d getCameraTransform(CameraSelect c) {
			return c == null ? null : CAMERA_TO_ROBOT_POSES[c.id];
		}

		public static final double getXYDeviation(double dx) {
			if(dx < 1) { return 0.01; }
			if(dx < 2) { return 0.02 * dx - 0.01; }	// from log data: deviation follows ~ 0.103x-0.157 (approximated here), but we also don't want negative values
			return 0.11 * dx - 0.19;
		}
		public static final double getXYZDeviation(double dx) {
			if(dx < 1) { return 0.01; }
			if(dx < 2) { return 0.04 * dx - 0.03; }
			return 0.14 * dx - 0.23;
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

		public static RawEstimationBuffer readEstimationQueue(RawEstimationBuffer buff) {
			if(buff == null) {
				buff = new RawEstimationBuffer();
			}
			buff.estimations = Vision.nt.combined_estimations.readQueue();
			buff.errors = Vision.nt.estimate_errors.readQueue();
			buff.distances = Vision.nt.tag_distances.readQueue();
			// handle size mismatches -- match arrays with the same timestamp
			return buff;
		}
		public static EstimationFrame[] extractEstimations(RawEstimationBuffer buff, EstimationFrame[] frames) {
			if(buff != null && buff.estimations != null) {
				final int len = buff.estimations.length;
				if(buff.errors.length != len || buff.distances.length != len) {
					return null;
				}
				if(frames == null || frames.length != len) {
					frames = new EstimationFrame[len];
				}
				for(int i = 0; i < len; i++) {
					frames[i] = new EstimationFrame();
					TimestampedDoubleArray posedata = buff.estimations[i];
					frames[i].tstamp = posedata.timestamp;
					Pose3d[] poses = extract3dPoses(posedata.value, null);
					frames[i].estimations = new Estimation[poses.length];
					for(int p = 0; p < poses.length; p++) {			// maybe assert that all the inner data lengths are the same
						frames[i].estimations[p] = new Estimation();
						frames[i].estimations[p].pose = poses[p];
						frames[i].estimations[p].rmse = buff.errors[i].value[p];		// or check during each loop
						frames[i].estimations[p].distance = buff.distances[i].value[p];
					}
				}
			}
			return frames;
		}

		// public static boolean insideBufferedFieldBounds(Estimation e, BoundingVolume v) {
		// 	return insideBuffered(e.pose, v, getXYZDeviation(e.distance));
		// }
		public static boolean insideBufferedFieldBounds(Estimation e, double flen, double fwidth) {
			return insideBufferedFieldBounds(e.pose, getXYZDeviation(e.distance), flen, fwidth);
		}
		public static boolean insideBufferedFieldBounds(Pose3d p, double dv, double flen, double fwidth) {
			return (
				(p.getX() > - dv) && (p.getX() < flen + dv) &&
				(p.getY() > - dv) && (p.getY() < fwidth + dv) &&
				(p.getZ() > - dv)
			);
		}

		// public static Estimation[] sortClosestNxN(Estimation[] e, int n, int s) {
		// 	Estimation[] ret = new Estimation[n];
		// 	for(int x = 0; x < n-1; x++) {
		// 		double min = Double.POSITIVE_INFINITY;
		// 		for(int b = (s*(x + 1)); b < (s*(x + 2)); b++) {
		// 			if(x > 0) {
		// 				double d = ret[x].pose.getTranslation().getDistance(e[b].pose.getTranslation());
		// 				if(d < min) {
		// 					ret[x + 1] = e[b];
		// 					min = d;
		// 				}
		// 			} else {
		// 				for(int a = s*x; a < (s*(x + 1)); a++) {
		// 					double d = e[a].pose.getTranslation().getDistance(e[b].pose.getTranslation());
		// 					if(d < min) {
		// 						ret[x] = e[a];
		// 						ret[x + 1] = e[b];
		// 						min = d;
		// 					}
		// 				}
		// 			}
		// 		}
		// 	}
		// 	return ret;
		// }




		public static void fuseVision(DriveBase db, boolean filter_bounds) {
			RawEstimationBuffer rbuff = PoseEstimation.readEstimationQueue(null);
			EstimationFrame[] frames = PoseEstimation.extractEstimations(rbuff, null);
			if(frames == null) { return; }
			TimestampedInteger apmode_ts = nt.aprilp_mode.getAtomic();
			for(EstimationFrame frame : frames) {
				if((frame.tstamp - apmode_ts.timestamp) * 1e6 < 0.15) {		// 150ms max delay between apmode change and detection up to date
					continue;
				}
				Transform3d c2r = getCameraTransform(
						selectedCameraFromID(
							aprilTagModeToCamID((int)apmode_ts.value)));

				Estimation best_rp = null;
				switch(frame.estimations.length) {
					case 0:
					default:
						continue;
					case 1:
					{
						best_rp = frame.estimations[0];
						best_rp.pose = best_rp.pose.transformBy(c2r);
						if(filter_bounds && !insideBufferedFieldBounds(best_rp, Field.FIELD_LENGTH, Field.FIELD_WIDTH)) {
							continue;
						}
						// the STDV seemed to be lower for megatag -- update 'distance'?
						break;
					}
					case 2:
					{
						Estimation a = frame.estimations[0];
						Estimation b = frame.estimations[1];
						a.pose.transformBy(c2r);
						b.pose.transformBy(c2r);
						double a_dv3 = getXYZDeviation(a.distance);
						double b_dv3 = getXYZDeviation(b.distance);
						// use field bound filtering
						if(filter_bounds) {
							boolean _a = insideBufferedFieldBounds(a.pose, a_dv3, Field.FIELD_LENGTH, Field.FIELD_WIDTH);
							boolean _b = insideBufferedFieldBounds(b.pose, b_dv3, Field.FIELD_LENGTH, Field.FIELD_WIDTH);
							if(!(_a || _b)) {
								continue;
							}
							if(_a && !_b) {
								best_rp = a;
								break;
							}
							if(_b && !_a) {
								best_rp = b;
								break;
							}
						}
						// apply the more statistically correct pose if margin is large enough
						if(a.rmse < b.rmse * 0.15) {
							best_rp = a;
							break;
						} else if(b.rmse < a.rmse * 0.15) {
							best_rp = b;
							break;
						}
						// if the poses are close enough to each other, apply both with increased empirical deviation
						double dx3 = a.pose.getTranslation().getDistance(b.pose.getTranslation());
						if(dx3 < a_dv3 || dx3 < b_dv3) {
							double cdist = (a.distance + b.distance);	// combine to get higher deviation
							db.setVisionStdDevs(getXYDeviation(cdist), getThetaDeviation(cdist));
							db.applyVisionUpdate(a.pose.toPose2d(), frame.tstamp / 1e6);
							db.applyVisionUpdate(b.pose.toPose2d(), frame.tstamp / 1e6);
							continue;
						}
						// compare to current pose, rotation, etc..?
						continue;	// skip frame if correct pose couldn't be deduced
					}
				}
				db.applyVisionUpdate(
					best_rp.pose.toPose2d(),
					frame.tstamp / 1e6,
					getXYDeviation(best_rp.distance),
					getThetaDeviation(best_rp.distance)
				);
			}
		}

	}

}
