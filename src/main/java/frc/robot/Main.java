// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
	private Main() {}

	public static void main(String... args) {
		// com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(10000.0);
		// UnmanagedJNI.JNI_SetPhoenixDiagnosticsStartTime(10000.0);
		RobotBase.startRobot(Robot::new);
	}
}
