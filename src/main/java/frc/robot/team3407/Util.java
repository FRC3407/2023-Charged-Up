package frc.robot.team3407;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Util {

	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t) {
		SmartDashboard.putData(t);
		return t;
	}
	/** Passthough for putting a sendable on SmartDashboard */
	public static <T extends Sendable> T send(T t, String key) {
		SmartDashboard.putData(key, t);
		return t;
	}

	/** Overload for {@link SenderNT} instances in place of the SmartDashboard singleton */
	public static <T extends Sendable> T send(T t, SenderNT sender) {
		sender.putData(t);
		return t;
	}
	/** Overload for {@link SenderNT} instances in place of the SmartDashboard singleton */
	public static <T extends Sendable> T send(T t, SenderNT sender, String key) {
		sender.putData(key, t);
		return t;
	}

	public static double zeroRange(double v, double epsilon) {
		return (v < epsilon && v > -epsilon) ? 0.0 : v;
	}
	public static double clamp(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}
	public static double clampEq(double v, double range) {
		return clamp(v, -Math.abs(range), Math.abs(range));
	}


	public static double[] toComponents2d(Pose2d... poses) {
		final double[] values = new double[poses.length * 3];
		for(int i = 0; i < poses.length; i++) {
			int offset = i * 3;
			values[offset + 0] = poses[i].getX();
			values[offset + 1] = poses[i].getY();
			values[offset + 2] = poses[i].getRotation().getRadians();
		}
		return values;
	}
	public static double[] toComponents3d(Pose3d... poses) {
		final double[] values = new double[poses.length * 7];
		for(int i = 0; i < poses.length; i++) {
			int offset = i * 7;
			Quaternion q = poses[i].getRotation().getQuaternion();
			values[offset + 0] = poses[i].getX();
			values[offset + 1] = poses[i].getY();
			values[offset + 2] = poses[i].getZ();
			values[offset + 3] = q.getW();
			values[offset + 4] = q.getX();
			values[offset + 5] = q.getY();
			values[offset + 6] = q.getZ();
		}
		return values;
	}

}
