// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double[] elevatorPositions = /*{0, 95, 134}*/{0, 3.5, 14.6};
  public static final double elevatorConversion = /*0.0368421053*/1;

  // Reef alignment constants
  public static final double X_REEF_ALIGNMENT_P = 3.3;
	public static final double Y_REEF_ALIGNMENT_P = 3.3;
	public static final double ROT_REEF_ALIGNMENT_P = 0.058;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

  // Vision alignment constants
  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;
}
