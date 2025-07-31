package frc.robot.subsystems.algaerollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class AlgaeRollers extends MotorSubsystem<MotorIOTalonFX> {
	public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
	public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(AlgaeRollerConstants.kIntakeVoltage);
	public static final Setpoint EXHAUST = Setpoint.withVoltageSetpoint(AlgaeRollerConstants.kExhaustVoltage);
	public static final Setpoint PROCESSOR = Setpoint.withVoltageSetpoint(AlgaeRollerConstants.kProcessorVoltage);
	public static final Setpoint L1_SOFT_SPIT = Setpoint.withVoltageSetpoint(AlgaeRollerConstants.kSoftL1Spit);

	public static final AlgaeRollers mInstance = new AlgaeRollers();

	private AlgaeRollers() {
		super(AlgaeRollerConstants.getMotorIO(), "Algae Rollers");
	}

	public Command getSpinForIntakeWithUnstuckCommand() {
		return new AlgaeIntakeWithBackupIfStuck();
	}

	public class AlgaeIntakeWithBackupIfStuck extends Command {
		private boolean intakeSetpointApplied = true;
		private Debouncer debounce = new Debouncer(0.5, DebounceType.kRising);
		private Timer exhaustTimer = new Timer();

		public AlgaeIntakeWithBackupIfStuck() {
			addRequirements(mInstance);
		}

		@Override
		public void initialize() {
			applyIntakeSetpoint();
			exhaustTimer.start();
		}

		private void applyIntakeSetpoint() {
			io.applySetpoint(INTAKE);
			intakeSetpointApplied = true;
		}

		private void applyExhaustSetpoint() {
			io.applySetpoint(EXHAUST);
			intakeSetpointApplied = false;
			exhaustTimer.reset();
		}

		private boolean getAboveStatorCurrentThreshold() {
			// return ControlBoardConstants.mDriverController.x().getAsBoolean();
			return AlgaeRollers.mInstance.getStatorCurrent().gte(Units.Amps.of(60.0));
		}

		@Override
		public void execute() {
			if (debounce.calculate(getAboveStatorCurrentThreshold()) && intakeSetpointApplied) {
				applyExhaustSetpoint();
			} else if (exhaustTimer.hasElapsed(0.2)) {
				applyIntakeSetpoint();
			}
		}
	}
}
