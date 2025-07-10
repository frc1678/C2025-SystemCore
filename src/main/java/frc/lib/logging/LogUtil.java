package frc.lib.logging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.stream.DoubleStream;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;

public class LogUtil {
	public static HashMap<String, StructPublisher<Pose2d>> structPublishers2dMap =
			new HashMap<String, StructPublisher<Pose2d>>();
	public static HashMap<String, StructPublisher<Pose3d>> structPublishers3dMap =
			new HashMap<String, StructPublisher<Pose3d>>();

	public static void logPose2d(String loggerName, Pose2d pose) {
		if (structPublishers2dMap.containsKey(loggerName)) {
			structPublishers2dMap.get(loggerName).set(pose);
		} else {
			StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
					.getStructTopic(loggerName, Pose2d.struct)
					.publish();
			structPublishers2dMap.put(loggerName, publisher);
		}
	}

	public static void logPose3d(String loggerName, Pose3d pose) {
		if (structPublishers3dMap.containsKey(loggerName)) {
			structPublishers3dMap.get(loggerName).set(pose);
		} else {
			StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
					.getStructTopic(loggerName, Pose3d.struct)
					.publish();
			structPublishers3dMap.put(loggerName, publisher);
		}
	}

	public static void recordTranslation2d(String key, Translation2d translation) {
		SmartDashboard.putNumberArray(key, new double[] {translation.getX(), translation.getY()});
	}

	public static void recordRotation2d(String key, Rotation2d rotation) {
		SmartDashboard.putNumber(key, rotation.getDegrees());
	}

	public static void recordPose2d(String key, Pose2d... poses) {
		final double[] doubleArray = Arrays.stream(poses)
				.flatMapToDouble(pose -> DoubleStream.of(
						pose.getTranslation().getX(),
						pose.getTranslation().getY(),
						pose.getRotation().getRadians()))
				.toArray();
		SmartDashboard.putNumberArray(key, doubleArray);
	}

	public static void recordPose3d(String key, Pose3d... poses) {
		final double[] doubleArray = Arrays.stream(poses)
				.flatMapToDouble(pose -> DoubleStream.of(
						pose.getTranslation().getX(),
						pose.getTranslation().getY(),
						pose.getTranslation().getZ(),
						pose.getRotation().getQuaternion().getW(),
						pose.getRotation().getQuaternion().getX(),
						pose.getRotation().getQuaternion().getY(),
						pose.getRotation().getQuaternion().getZ()))
				.toArray();
		SmartDashboard.putNumberArray(key, doubleArray);
	}

	public static void recordTrajectory(String key, Trajectory trajectory) {
		ArrayList<Pose2d> poses = new ArrayList<>();
		for (int i = 1; i < trajectory.getStates().size(); ++i) { // Don't send all poses to save performance
			poses.add(trajectory.getStates().get(i).pose);
		}
		recordPose2d(key, poses.toArray(new Pose2d[0]));
	}

	public static void recordTrajectory(String key, choreo.trajectory.Trajectory<?> trajectory) {
		ArrayList<Pose2d> poses = new ArrayList<>();
		for (int i = 1; i < trajectory.samples().size(); ++i) {
			poses.add(trajectory.samples().get(i).getPose());
		}
		recordPose2d(key, poses.toArray(new Pose2d[0]));
	}

	/**
	 *
	 * @param key Logged poses will be called 'key + " confidence interval a1...b2"'
	 * @param confidence Probablity robot is within the ellipse
	 * @param covMatrix Covariance matrix of measurement
	 * @param x x-value of measurement
	 * @param y y-value of measurement
	 *
	 */
	public static void logConfidenceEllipse(
			String key, double confidence, Matrix<N2, N2> covMatrix, double x, double y) {

		SimpleEVD<SimpleMatrix> deconstructor =
				new SimpleEVD<SimpleMatrix>(covMatrix.getStorage().getMatrix());
		List<Complex_F64> eigenvalues = deconstructor.getEigenvalues();
		if (eigenvalues.get(0).imaginary != 0 | eigenvalues.get(1).imaginary != 0) {
			System.out.println("Not real");
		} else {
			double a;
			double b;
			double scaleFactor = Math.sqrt(-2 * Math.log(1 - confidence));
			// SimpleMatrix eigVec;

			if (eigenvalues.get(0).real > eigenvalues.get(1).real) {
				a = scaleFactor * Math.sqrt(eigenvalues.get(0).real);
				b = scaleFactor * Math.sqrt(eigenvalues.get(1).real);

				// eigVec = deconstructor.getEigenVector(0);
			} else {
				b = scaleFactor * Math.sqrt(eigenvalues.get(0).real);
				a = scaleFactor * Math.sqrt(eigenvalues.get(1).real);

				// eigVec = deconstructor.getEigenVector(1);
			}
			// double theta = Math.atan(eigVec.get(1)/eigVec.get(0)) % (2*Math.PI);
			// (theta = PI/2)
			Pose2d a1 = new Pose2d(new Translation2d(x, y + a), Rotation2d.fromDegrees(90));
			Pose2d a2 = new Pose2d(new Translation2d(x, y - a), Rotation2d.fromDegrees(270));
			Pose2d b1 = new Pose2d(new Translation2d(x - b, y), Rotation2d.fromDegrees(180));
			Pose2d b2 = new Pose2d(new Translation2d(x + b, y), Rotation2d.fromDegrees(0));

			recordPose2d(key + " confidence interval a1", a1);
			recordPose2d(key + " confidence interval a2", a2);
			recordPose2d(key + " confidence interval b1", b1);
			recordPose2d(key + " confidence interval b2", b2);
		}
	}

	public static final class GcStatsCollector {
		private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
		private final long[] lastTimes = new long[gcBeans.size()];
		private final long[] lastCounts = new long[gcBeans.size()];

		public void update() {
			long accumTime = 0;
			long accumCounts = 0;
			for (int i = 0; i < gcBeans.size(); i++) {
				long gcTime = gcBeans.get(i).getCollectionTime();
				long gcCount = gcBeans.get(i).getCollectionCount();
				accumTime += gcTime - lastTimes[i];
				accumCounts += gcCount - lastCounts[i];

				lastTimes[i] = gcTime;
				lastCounts[i] = gcCount;
			}

			SmartDashboard.putNumber("Logged Robot/GCTimeMS", (double) accumTime);
			SmartDashboard.putNumber("Logged Robot/GCCounts", (double) accumCounts);
		}
	}
}
