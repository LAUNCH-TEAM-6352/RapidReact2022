// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static final class DriveTrainConstants
	{
		public static final int leftFrontMotorChannel = 10;
		public static final int leftRearMotorChannel = 11;
		public static final int leftTopMotorChannel = 12;
		public static final int rightFrontMotorChannel = 13;
		public static final int rightRearMotorChannel = 14;
		public static final int rightTopMotorChannel = 15;

        public static final int[] leftMotorChannels = {leftFrontMotorChannel, leftRearMotorChannel, leftTopMotorChannel};
        public static final int[] rightMotorChannels = {rightFrontMotorChannel, rightRearMotorChannel, rightTopMotorChannel};

		public static final boolean leftMotorsInverted = true;
		public static final boolean rightMotorsInverted = false;

        public static final double defaultPercentage = 0.5;
	}
    
	public static final class OIConstants
	{
		public static final int xboxControllerPort = 1;
		public static final int leftJoystickPort = 2;
		public static final int rightJoystickPort = 3;
	}

    public static final class DashboardConstants
	{
		public static final String driveTrainPercentageKey = "Drive Train %";
		public static final String driveTrainPositionKey = "Drive Train Pos";
	}

}
