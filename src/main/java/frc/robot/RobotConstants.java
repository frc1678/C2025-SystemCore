package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;

public class RobotConstants {
	public static String kSerial;
	public static boolean isComp;
	public static boolean isOmega;

	public static final double lowBatteryVoltage = 11.8; 
	public static final Time kBatteryAlertDisabledTime = Units.Seconds.of(2.0);
	public static final long kLowBatteryMinLoopCount = 10; 

	static {
		RobotConstants.isComp = false;
		RobotConstants.isOmega = true;
		RobotConstants.isRedAlliance = false;
	}

	public static boolean isRedAlliance;
	public static AutoFactory mAutoFactory;

	public static final String kCompSerial = "03415A0E";
	public static final String kOmegaSerial = "032B4B47";
}
