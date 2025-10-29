package frc.robot.trajectories;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.MovingAveragePose2d;
import frc.lib.util.Stopwatch;
import frc.lib.util.Util;
import frc.robot.autos.AutoConstants;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class DetectionPIDToPoseCommand extends PIDToPoseCommand {
	private final Pose2d defaultFinalPose;
	private Stopwatch timer = new Stopwatch();
	private Stopwatch debounce = new Stopwatch();

	private AutoType side;
	private boolean firstUpdate; // stores when the final pose was first updated
	private Rotation2d firstRotation; // stores what the rotation of the robot was during the first update

	private MovingAveragePose2d movingAveragePose2d =
			new MovingAveragePose2d(15); // more likely to trust new poses but still keeps track of a few older ones

	public DetectionPIDToPoseCommand(Pose2d defaultFinalPose, AutoType side) {
		this(defaultFinalPose, side, false);
	}

	public DetectionPIDToPoseCommand(Pose2d defaultFinalPose, AutoType side, boolean quick) {
		super(
				defaultFinalPose,
				quick ? AutoConstants.getDetectionQuickTranslationController() : AutoConstants.getDetectionTranslationController(),
				AutoConstants.getDetectionHeadingController());
		this.defaultFinalPose = defaultFinalPose;
		this.side = side;
		this.epsilonDist = Units.Inches.of(4.0);
		this.epsilonAngle = Units.Degrees.of(4.0);
	}


	public DetectionPIDToPoseCommand(AutoTrajectory trajectory, AutoType side) {
		this(trajectory.getFinalPose().get(), side, false);
	}

	public DetectionPIDToPoseCommand(AutoTrajectory trajectory, AutoType side, boolean quick) {
		this(trajectory.getFinalPose().get(), side, quick);
	}

	@Override
	public void initialize() {
		firstUpdate = false;
		finalPose = defaultFinalPose;
		movingAveragePose2d.clear();
		timer.startIfNotRunning();
		super.initialize();
	}

	@Override
	public void execute() {
		if (Detection.mInstance.hasCoral()) {
			debounce.startIfNotRunning();
			if (debounce.getTime().gte(Units.Milliseconds.of(40.0))
					&& !firstUpdate) { // avoid overrotating by checking what OG value was first
				firstRotation =
						Detection.mInstance.getCoralTranslationAndPoint().getRotation();
				firstUpdate = true;
				debounce.reset();
			}

			Rotation2d rot = Detection.mInstance
					.getCoralTranslationAndPoint()
					.getTranslation()
					.minus(Drive.mInstance.getPose().getTranslation())
					.getAngle();
			Translation2d finalTranslation = Detection.mInstance
					.getCoralTranslationAndPoint()
					.getTranslation()
					.minus(new Translation2d(edu.wpi.first.math.util.Units.feetToMeters(0.0), rot));

			Pose2d newPose = new Pose2d(
					finalTranslation,
					Detection.mInstance.getCoralTranslationAndPoint().getRotation());

			if (side == AutoType.MARK) {
				if (firstUpdate) {
					movingAveragePose2d.add(newPose);
					finalPose = movingAveragePose2d.getAverage();
				}
			} else {
				// when there is a first update, start adding poses
				// if the new pose is within the zone, add
				// if the new pose is NOT near a mark, add
				// if the new pose wouldn't require you to turn more than 40 degrees to get to from your current
				// heading, add
				// if the new pose is within a foot of what was in the moving average, add (also add if there was no
				// update to it in the first place)
				// if the drivetrain is NOT near the final pose, add (if the drivetrain is really close to the final
				// pose, STOP adding tracks)
				if (firstUpdate
						&& FieldLayout.getIsInZone(newPose.getTranslation(), side)
						&& !FieldLayout.nearMark(newPose)
						&& Util.epsilonEquals(
								Drive.mInstance.getPose().getRotation().getDegrees(),
								Detection.mInstance
										.getCoralTranslationAndPoint()
										.getRotation()
										.getDegrees(),
								60)
						&& !Util.epsilonEquals(
								Drive.mInstance.getPose().getTranslation(),
								finalPose.getTranslation(),
								Units.Feet.of(0.0))) {
					movingAveragePose2d.add(newPose);
					finalPose = movingAveragePose2d.getAverage();
					SmartDashboard.putNumber("Detection PID/Last Accepted Update", Timer.getFPGATimestamp());
				}
			}

			SmartDashboard.putBoolean(
					"Detection PID/Is In Zone", FieldLayout.getIsInZone(newPose.getTranslation(), side));
			SmartDashboard.putBoolean("Detection PID/Not Near Mark", !FieldLayout.nearMark(newPose));
			SmartDashboard.putBoolean(
					"Detection PID/Heading Is Similar",
					Util.epsilonEquals(
							Drive.mInstance.getPose().getRotation().getDegrees(),
							Detection.mInstance
									.getCoralTranslationAndPoint()
									.getRotation()
									.getDegrees(),
							60));
			SmartDashboard.putBoolean(
					"Detection PID/Detection And Moving Avg Close",
					Util.epsilonEquals(
							newPose.getTranslation(),
							movingAveragePose2d.getAverage().getTranslation(),
							Units.Feet.of(1.0)));
			SmartDashboard.putBoolean(
					"Detection PID/Drivetrain Not Near Final Pose",
					!Util.epsilonEquals(
							Drive.mInstance.getPose().getTranslation(),
							finalPose.getTranslation(),
							Units.Feet.of(0.2)));

			// SmartDashboard.putNumber("Detection PID/Debounce Time", debounce.getTimeAsDouble());
			SmartDashboard.putBoolean("Detection PID/First Update", firstUpdate);

			SmartDashboard.putNumber(
					"Detection PID/Epsilon",
					newPose.getTranslation()
							.getDistance(movingAveragePose2d.getAverage().getTranslation()));
			// LogUtil.recordRotation2d("Detection PID/First Rotation", firstRotation);
			LogUtil.recordPose2d("Detection PID/Moving Average Pose", movingAveragePose2d.getAverage());
		}
		super.execute();
	}

	@Override
	public void end(boolean interrupted) {
		timer.reset();
		debounce.reset();
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return super.isFinished()
				|| Superstructure.indexerBreak.getDebouncedIfReal()
				|| timer.getTime().gte(Units.Seconds.of(3.5));
	}
}
