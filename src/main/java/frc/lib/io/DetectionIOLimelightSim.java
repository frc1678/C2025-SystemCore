package frc.lib.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import frc.lib.util.Stopwatch;
import frc.robot.subsystems.drive.Drive;

/**
 * Does nothing, used for simulation.
 */
public class DetectionIOLimelightSim extends DetectionIOLimelight {
	private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private final NetworkTable visTable = ntInstance.getTable("SmartDashboard/Detection");
	private final StructPublisher<Pose2d> closestCoralPose =
			visTable.getStructTopic("CoralPose", Pose2d.struct).publish();
	private final StructPublisher<Translation2d> closestCoralTranslation =
			visTable.getStructTopic("CoralTranslation", Translation2d.struct).publish();

	private final Stopwatch timer = new Stopwatch();

	@Override
	public Pose2d getCoralPose(Translation2d base) {
		// Feed in the coral pose you want the robot to track to here
		timer.startIfNotRunning();
		if (timer.getTime().gte(Units.Seconds.of(4))) {
			Pose2d coralPose = new Pose2d(14.0, 7.23, Drive.mInstance.getPose().getRotation());
			if (timer.getTime().gte(Units.Seconds.of(6)))
				coralPose = new Pose2d(14.3, 7.23, Drive.mInstance.getPose().getRotation());
			if (timer.getTime().gte(Units.Seconds.of(8)))
				coralPose = new Pose2d(14.6, 7.23, Drive.mInstance.getPose().getRotation());
			if (timer.getTime().gte(Units.Seconds.of(10)))
				coralPose = new Pose2d(14.9, 7.23, Drive.mInstance.getPose().getRotation());
			if (timer.getTime().gte(Units.Seconds.of(12)))
				coralPose = new Pose2d(15.2, 7.23, Drive.mInstance.getPose().getRotation());
			if (timer.getTime().gte(Units.Seconds.of(14)))
				coralPose = new Pose2d(15.5, 7.23, Drive.mInstance.getPose().getRotation());
			closestCoralPose.set(coralPose);
			return coralPose;
		}
		return null;
	}

	@Override
	public int coralCount() {
		timer.startIfNotRunning();
		return timer.getTime().gte(Units.Seconds.of(4)) ? 1 : 0;
	}
}
