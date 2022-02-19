// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
	public static final class OIConstants
	{
		public static final int xboxControllerPort = 1;
		public static final int leftJoystickPort = 2;
		public static final int rightJoystickPort = 3;
	}
    
	public static final class DriveTrainConstants
	{
        // Motor controllers are SPARK MAX:
		public static final int leftFrontMotorChannel = 14;
		public static final int leftRearMotorChannel = 13;
		public static final int leftTopMotorChannel = 15;
		public static final int rightFrontMotorChannel = 11;
		public static final int rightRearMotorChannel = 10;
		public static final int rightTopMotorChannel = 12;

        //public static final int[] leftMotorChannels = {leftFrontMotorChannel, leftRearMotorChannel, leftTopMotorChannel};
        //public static final int[] rightMotorChannels = {rightFrontMotorChannel, rightRearMotorChannel, rightTopMotorChannel};
        public static final int[] leftMotorChannels = {};
        public static final int[] rightMotorChannels = {};

        // Indicates if motor controller should output negative of commanded percentage:
		public static final boolean isLeftMotorInverted = true;
		public static final boolean areRightMotorsInverted = false;

        // Indicates if encoder signal is inverted:
        public static final boolean isLeftEncoderInverted = false;
        public static final boolean isRightEncoderInverted = false;

        // If motors shjould coast or brake to a stop:
        public static final IdleMode idleMode = IdleMode.kCoast;

        // Values for PID controller used for driving to a specific position:
        public static final double pidP = 0.08;
        public static final double pidI = 0.00001;
        public static final double pidD = 1.0;
        public static final double pidIZ = 10.0;
        public static final double pidFF = 0.0;
        public static final double pidMaxOutput = +0.5;
        public static final double pidMinOutput = -0.5;

        // Default values for Smart Dashboard:
        public static final double defaultPercentage = 0.5;
        public static final double defaultOpenLoopRampRate = 0.0;
        public static final double defaultClosedLoopRampRate = 0.0;
	}
    
	public static final class PneumaticsConstants
	{
		public static final PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;

        public static final int intakeInChannel = 0;
        public static final int intakeoutChannel = 1;

        public static final int tbdAChannel = 2;
        public static final int tbdBChannel = 3;
	}
    
    public static final class IndexerConstants
    {
        // Motor controllers are Victor SPX:
        public static final int upperMotorChannel = 24;
        public static final int lowerMotorChannel = 25;
    }

    public static final class IntakeConstants
    {
        // Motor controller is Victor SPX:
        public static final int intakeMotorChannel = 26;
    }

	public static final class ShooterConstants
	{
        // Motor controllers are Talon SRX:
		public static final int leftMotorChannel = 22;
		public static final int rightMotorChannel = 23;
		public static final boolean isLeftMotorInverted = false;
		public static final boolean isRightMotorInverted = true;

		public static final double defaultLowVelocity = 1800;
		public static final double defaultHighVelocity = 2500;
		public static final double defaultPercentage = 0.5;

        // This is the max output of the 4:1 gearbox.
        // This equates to about 3,500 RPM.
		public static final double peakVelocityUnitsPer100ms = 23800.0;

        // PID constants:
		public static final int pidProfileSlot = 0;
		public static final double pidP = 0.8;
		public static final double pidI = 0.00001;
		public static final double pidD = 0.03;
		public static final int pidIZ = 3000;
		public static final double pidFF = 1023.0 / peakVelocityUnitsPer100ms;
		public static final double pidPeakOutput = 1;
		public static final int pidLoopPeriodMs = 1;
		public static final double pidMaxIntegralAccum = 0;
		public static final int pidAllowableError = 0;
		public static final int pidTimeoutMs = 30;

        // Encoder constants:
		public static final int countsPerRevolution = 1024;
		public static final int ticksPerCount = 4;
		public static final int primaryClosedLoop = 0;
		public static final boolean isSensorPhaseInverted = false;

        // Indicates if motors should coast or brake to a stop:
        public static final NeutralMode neutralMode = NeutralMode.Coast;
	}

    public static final class DashboardConstants
	{
		public static final String driveTrainPercentageKey = "Drive Train %";
		public static final String driveTrainLeftPositionKey = "DT Left Pos";
		public static final String driveTrainRightPositionKey = "DT Right Pos";
		public static final String driveTrainOpenLoopRampRateKey = "OL Ramp Rate (secs)";
		public static final String driveTrainClosedLoopRampRateKey = "CL Ramp Rate (secs)";
        public static final String driveTrainLeftPercentOutput = "DT Left % Output";
        public static final String driveTrainRightPercentOutput = "DT Right % Output";

		public static final String shooterLowTargetVelocityKey = "Shooter Low RPM";
		public static final String shooterHighTargetVelocityKey = "Shooter High RPM";
		public static final String shooterSetVelocityKey = "Shooter Vel Set";
		public static final String shooterCurrentVelocityLeftKey = "Shooter RPM Left";
		public static final String shooterCurrentVelocityRightKey = "Shooter RPM Rght";
        public static final String shooterRampUpTimeKey = "Shooter Ramp Secs";

        public static final String shooterTargetPercentageKey = "Shooter %";
    }

}
