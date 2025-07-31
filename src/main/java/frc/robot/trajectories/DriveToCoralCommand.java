package frc.robot.trajectories;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Stopwatch;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class DriveToCoralCommand extends Command {
	private Stopwatch timer = new Stopwatch();
	Pose2d target = null;
	boolean notSeenCoral;

	public DriveToCoralCommand() {
		addRequirements(Drive.mInstance);
	}

	@Override
	public void initialize() {
		timer.start();
		super.initialize();
		target = Detection.mInstance.getCoralPose(Drive.mInstance.getPose().getTranslation());
		notSeenCoral = target == null;
	}

	@Override
	public void execute() {
		if (!notSeenCoral) {
			Drive.mInstance.setSwerveRequest(DriveConstants.getPIDToTranslationForwardRequestUpdater(target)
					.apply(DriveConstants.PIDToPoseRequest));
		} else {
			Drive.mInstance.setSwerveRequest(new SwerveRequest.RobotCentric().withVelocityX(0.5));
			target = Detection.mInstance.getCoralPose(Drive.mInstance.getPose().getTranslation());
			notSeenCoral = target == null;
		}
	}

	@Override
	public void end(boolean interrupted) {
		Drive.mInstance.followSwerveRequestCommand(DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater);
	}

	@Override
	public boolean isFinished() {
		return timer.getTimeAsDouble() > 3.5 || Superstructure.mInstance.getEndEffectorCoralBreak();
	}
}
