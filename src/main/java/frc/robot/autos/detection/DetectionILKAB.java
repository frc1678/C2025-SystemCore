package frc.robot.autos.detection;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.robot.autos.AutoConstants.AutoEndBehavior;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeBase;
import frc.robot.subsystems.algaedeploy.AlgaeDeploy;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.detection.DetectionConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class DetectionILKAB extends AutoModeBase {
	public DetectionILKAB(AutoFactory factory) {
		super(factory, "Detection ILKAB", AutoType.LEFT);

		Superstructure s = Superstructure.mInstance;
		Detection d = Detection.mInstance;

		AutoTrajectory rightStartToI = trajectory("rightStartToI");

		prepRoutine(
				AutoHelpers.resetPoseIfWithoutEstimate(
						rightStartToI.getInitialPose().get()),
				rightStartToI
						.cmd()
						.alongWith(d.setPipelineCmd(DetectionConstants.kAutoPipeline))
						.andThen(autoScoreWithPrepWithoutCoralHold(Branch.I, Level.L4)),
				intakeAndScoreGroundCoralQuick("iToDetection", Branch.L, Level.L4_QUICK, false),
				intakeAndScoreGroundCoralQuick("lToDetection", Branch.K, Level.L4_QUICK, false),
				intakeAndScoreGroundCoralQuick("kToDetection", Branch.A, Level.L4_QUICK, true),
				intakeAndScoreMark("aToMidMark", Branch.B, Level.L4, false));
	}
}