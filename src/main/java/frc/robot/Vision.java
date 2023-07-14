package frc.robot;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.util.Units;
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
			this.active_cam =			this.vbase.getIntegerTopic("Active Camera Thread")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.overlay_verbosity =	this.vbase.getIntegerTopic("Overlay Verbosity")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.cam_exposure =			this.vbase.getIntegerTopic("Camera Exposure")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.downscaling =			this.vbase.getIntegerTopic("Output Downscale")
											.getEntry(0, Vision.NT_ENTRY_DEFAULT);
			this.aprilp_mode =			this.vbase.getIntegerTopic("AprilTag Mode")
											.getEntry(-2, Vision.NT_ENTRY_DEFAULT);
			this.retror_mode =			this.vbase.getIntegerTopic("RetroRefl Mode")
											.getEntry(-2, Vision.NT_ENTRY_DEFAULT);
			this.perf_override = 		this.vbase.getIntegerTopic("Performance Mode")
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
			this.retrorefl_locations =	this.vbase.getDoubleArrayTopic("RetroReflective Detections/Translations")
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
			active_cam,
			overlay_verbosity,
			cam_exposure,
			downscaling,
			aprilp_mode,
			retror_mode,
			perf_override;
		public final DoubleArraySubscriber
			all_estimations,
			combined_estimations,
			retrorefl_centers,
			retrorefl_locations;
		public final FloatArraySubscriber
			estimate_errors,
			tag_distances;
	}

	public static NT nt = null;

	static void init() {
		nt = NT.get();
	}


	public static enum CameraSelect {
		UNDERARM		(0, "Under Arm Camera", PoseEstimation.UNDER_ARM_C2R),
		UPPER			(1, "Upper Camera", PoseEstimation.UPPER_C2R);

		public final int id;
		public final String name;
		public final Transform3d c2r;
		private CameraSelect(int id, String n, Transform3d c2r) {
			this.id = id;
			this.name = n;
			this.c2r = c2r;
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

		public void setActive() { setVerbosity(this); }
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

	public static enum PerfMode {
		STREAM_ONLY			(-1),
		NONE				(0),
		RAW_MULTISTREAM		(1),
		SINGLESTREAM		(2),
		RAW_SINGLESTREAM	(3),
		VPP_ONLY			(4),
		APRIL_ONLY			(5),
		RETRO_ONLY			(6);

		public final int val;
		private PerfMode(int v) { this.val = v; }

		public void setActive() { setPerfMode(this.val); }
	}
	public static void setPerfMode(PerfMode p) {
		nt.perf_override.set(p.val);
	}
	public static void setPerfMode(int m) {
		nt.perf_override.set(m);
	}
	public static int getPerfMode() {
		return (int)nt.perf_override.get();
	}
	public static PerfMode perfModeFromVal(int v) {
		final PerfMode[] vals = PerfMode.values();
		if(v < -1) { return PerfMode.STREAM_ONLY; }
		else if(v <= PerfMode.RETRO_ONLY.val) { return vals[v + 1]; }
		return null;
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
				sel_under,
				sel_upper;

			public DirectSwitching(BooleanSupplier und, BooleanSupplier up) { this(()->false, ()->false, ()->false, und, up); }
			public DirectSwitching(BooleanSupplier und, BooleanSupplier up, BooleanSupplier vrb) { this(()->false, ()->false, vrb, und, up); }
			public DirectSwitching(
				BooleanSupplier inc, BooleanSupplier vrb,
				BooleanSupplier und, BooleanSupplier up
			) { this(inc, ()->false, vrb, und, up); }
			public DirectSwitching(
				BooleanSupplier inc, BooleanSupplier dec, BooleanSupplier vrb,
				BooleanSupplier und, BooleanSupplier up
			) {
				super(inc, dec, vrb);
				this.sel_under = und;
				this.sel_upper = up;
			}

			@Override
			public void execute() {
				super.execute();
				if(this.sel_under.getAsBoolean()) { Vision.selectCamera(super.selected = CameraSelect.UNDERARM); }
				if(this.sel_upper.getAsBoolean()) { Vision.selectCamera(super.selected = CameraSelect.UPPER); }
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

		public static final Transform3d		// the inverses of the camera locations in robot coord space -- +x is "forward", +y is right, +z is up
			UNDER_ARM_C2R = new Transform3d(
				new Translation3d(
					+0.075431,	// forward amount
					-0.004467,	// rightward amount
					+0.936205),	// upward amount
				new Rotation3d(0.0, 10.927535, 0.0)
			).inverse(),
			UPPER_C2R = new Transform3d(
				new Translation3d(
					+0.271950,	// forward amount
					-0.106706,	// rightward amount
					+0.994151),	// upward amount
				new Rotation3d(0.0, 12.0, 0.0)
			).inverse();

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
			final int len = raw.length / 7;
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




		public static void fuseVision(DriveBase db, boolean filter_bounds) {
			RawEstimationBuffer rbuff = PoseEstimation.readEstimationQueue(null);
			EstimationFrame[] frames = PoseEstimation.extractEstimations(rbuff, null);
			if(frames == null || frames.length == 0) { return; }
			TimestampedInteger apmode_ts = nt.aprilp_mode.getAtomic();
			for(EstimationFrame frame : frames) {
				if((frame.tstamp - apmode_ts.timestamp) * 1e6 < 0.15) {		// 150ms max delay between apmode change and detection up to date
					continue;
				}
				Transform3d c2r = selectedCameraFromID(aprilTagModeToCamID((int)apmode_ts.value)).c2r;

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

	public static final class RetroDetection {

		public static Translation3d[] extractTranslations(double[] raw, Translation3d[] buff) {
			final int len = raw.length / 3;
			if(buff == null || buff.length != len) {
				buff = new Translation3d[len];
			}
			for(int i = 0; i / 3 < buff.length;) {
				buff[i / 3] = new Translation3d(
					raw[i++],
					raw[i++],
					raw[i++]
				);
			}
			return buff;
		}

	}

}
