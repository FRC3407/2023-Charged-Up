package frc.robot.team3407.drive;

import java.util.function.DoubleSupplier;


public final class DriveSupplier {

	/** A supplier of l/r wheel speeds for a differential drivebase */
	public static interface DifferentialDriveSupplier {
		public static class CombinedOutput {
			public CombinedOutput(double l, double r) { this.left = l; this.right = r; }
			public CombinedOutput() {}
			public double left, right;
		}

		public double leftOutput();
		public double rightOutput();
		default public CombinedOutput getOutputs() { return this.getOutputs(new CombinedOutput()); }
		default public CombinedOutput getOutputs(CombinedOutput o) {
			o.left = this.leftOutput();
			o.right = this.rightOutput();
			return o;
		}
	}

	/** Suppliers that always return the same value. Provides helpers for different generation types */
	public static class StaticSupplier implements DifferentialDriveSupplier {
		public final double l, r;
		public StaticSupplier (double l, double r) {
			this.l = l;
			this.r = r;
		}

		@Override
		public double leftOutput() { return this.l; }
		@Override
		public double rightOutput() { return this.r; }

		public static StaticSupplier genSimple(double fb) { return new StaticSupplier(fb, fb); }
		public static StaticSupplier genTank(double l, double r) { return new StaticSupplier(l, r); }
		public static StaticSupplier genArcade(double fb, double t) { return new StaticSupplier(fb + t, fb - t); }
	}
	/** Default tank input converter that simply passes the values supplied */
	public static class TankSupplier implements DifferentialDriveSupplier {
		protected final DoubleSupplier l, r;
		public TankSupplier(DoubleSupplier l, DoubleSupplier r) {
			this.l = l;
			this.r = r;
		}

		@Override
		public double leftOutput() { return this.l.getAsDouble(); }
		@Override
		public double rightOutput() { return this.r.getAsDouble(); }
	}
	/** Tank input converter that limits the rotational rate using the supplier provided */
	public static class TankSupplierRS extends TankSupplier {
		protected final DoubleSupplier rs;
		public TankSupplierRS(DoubleSupplier l, DoubleSupplier r, DoubleSupplier rs) {
			super(l, r);
			this.rs = rs;
		}

		@Override
		public double leftOutput() { return this.getOutputs().left; }
		@Override
		public double rightOutput() { return this.getOutputs().right; }
		@Override
		public CombinedOutput getOutputs(CombinedOutput o) {
			double
				l = super.l.getAsDouble(),
				r = super.r.getAsDouble(),
				avg = (l + r) / 2.0,
				off = (l - r) / 2.0 * this.rs.getAsDouble();
			o.left = avg + off;
			o.right = avg - off;
			return o;
		}
	}
	/** Arcade input converter that simply adds the turning rate to the f/b rate for each side */
	public static class ArcadeSupplier implements DifferentialDriveSupplier {
		protected final DoubleSupplier f, t;
		public ArcadeSupplier(DoubleSupplier f, DoubleSupplier t) {
			this.f = f;
			this.t = t;
		}

		@Override
		public double leftOutput() {
			return this.f.getAsDouble() + this.t.getAsDouble();
		}
		@Override
		public double rightOutput() {
			return this.f.getAsDouble() - this.t.getAsDouble();
		}
	}
	/** Arcade input converter that limits either side to the provided maximum output magnitude, and prioritizes turning rate over f/b velocity */
	public static class ArcadeSupplierLM extends ArcadeSupplier {
		protected final DoubleSupplier max;
		public ArcadeSupplierLM(DoubleSupplier f, DoubleSupplier t, DoubleSupplier max) {
			super(f, t);
			this.max = max;
		}

		@Override
		public double leftOutput() { return this.getOutputs().left; }
		@Override
		public double rightOutput() { return this.getOutputs().right; }
		@Override
		public CombinedOutput getOutputs(CombinedOutput o) {
			double
				f = super.f.getAsDouble(),
				t = super.t.getAsDouble(),
				m = Math.abs(f) + Math.abs(t) - Math.abs(this.max.getAsDouble());
			if(m > 0) {
				o.left = f + t - Math.copySign(m, f);
				o.right = f - t - Math.copySign(m, f);
			} else {
				o.left = f + t;
				o.right = f - t;
			}
			return o;
		}
	}


}
