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
		public static final int gamepadPort = 1;
		public static final int leftJoystickPort = 2;
		public static final int rightJoystickPort = 3;
	}
    
	public static final class DriveTrainConstants
	{
        /**
         * Used to give names to motors.
         */
        public static class MotorName
        {
            public final int channel;
            public final String name;
            public final String abbreviation;

            public MotorName (int channel, String name, String abbreviation)
            {
                this.channel = channel;
                this.name = name;
                this.abbreviation = abbreviation;
            }
        }

        // Motor controllers are SPARK MAX:
		public static final int leftFrontMotorChannel = 14;
		public static final int leftRearMotorChannel = 13;
		public static final int leftTopMotorChannel = 15;
		public static final int rightFrontMotorChannel = 11;
		public static final int rightRearMotorChannel = 10;
		public static final int rightTopMotorChannel = 12;

        public static final int[] leftMotorChannels = {leftFrontMotorChannel, leftRearMotorChannel, leftTopMotorChannel};
        public static final int[] rightMotorChannels = {rightFrontMotorChannel, rightRearMotorChannel, rightTopMotorChannel};
        // public static final int[] leftMotorChannels = {leftRearMotorChannel};
        // public static final int[] rightMotorChannels = {rightRearMotorChannel};

        public static final MotorName[] motorNames =
        {
            new MotorName(leftFrontMotorChannel, "Left Front", "LF"),
            new MotorName(leftTopMotorChannel, "Left Top", "LT"),
            new MotorName(leftRearMotorChannel, "Left Rear", "LR"),
            new MotorName(rightFrontMotorChannel, "Right Front", "RF"),
            new MotorName(rightTopMotorChannel, "Right Top", "RT"),
            new MotorName(rightRearMotorChannel, "Right Rear", "RR")
        };

        // Indicates if motor controller should output negative of commanded percentage:
		public static final boolean isLeftMotorInverted = true;
		public static final boolean areRightMotorsInverted = false;

        // Indicates if encoder signal is inverted:
        public static final boolean isLeftEncoderInverted = false;
        public static final boolean isRightEncoderInverted = false;

        // If motors shjould coast or brake to a stop:
        public static final IdleMode idleMode = IdleMode.kCoast;

        public static final double defaultAutoTargetPosition = -1000;

        // Tolerance for determining if at target position:
        public static final double positionTolerance = 10;

        // Default values for PID controller used for driving to a specific position:
        public static final double defaultPidP = 0.08;
        public static final double defaultPidI = 0.00001;
        public static final double defaultPidD = 1.0;
        public static final double defaultPidIZ = 10.0;
        public static final double defaultPidFF = 0.0;
        public static final double defaultPidMinOutput = -0.5;
        public static final double defaultPidMaxOutput = +0.5;

        // Default values for Smart Dashboard:
        public static final double defaultPercentage = 0.5;
        public static final double defaultOpenLoopRampRate = 1.0;
        public static final double defaultClosedLoopRampRate = 2.0;
	}
    
	public static final class PneumaticsConstants
	{
        public static final int moduleId = 0;
		public static final PneumaticsModuleType moduleType = PneumaticsModuleType.CTREPCM;
	}
    
	public static final class ClimberConstants
	{
        // Motor controllers are Victor SPX:
        public static final int leftMotorChannel = 27;
        public static final int rightMotorChannel = 28;
        public static final boolean isLeftMotorInverted = false;
        public static final boolean isRightMotorInverted = true;
        public static final double defaultHookExtendSpeed = 0.30;
        public static final double defaultHookRetractSpeed = -1.00;
        public static final NeutralMode motorNeutralMode = NeutralMode.Brake;

        // Theseare the pneumartics chnnels for moving the climber mechanism uip and down:
        public static final int pneumaticsUpChannel = 0;
        public static final int pneumaticsDownChannel = 1;

        // These aere the digital I/O channels for limit switches:
		public static final int extendLimitChannel = 0; 
		public static final int retractLimitChannel = 1; 
	}
    
    public static final class IndexerConstants
    {
        // Motor controllers are Victor SPX:
        public static final int upperMotorChannel = 24;
        public static final int lowerMotorChannel = 25;

        // Want to use positive percentage values for "in".
        // May need to invert those to get the motors running in the right directions:
        public static boolean isUpperMotorInverted = false;
        public static boolean isLowerMotorInverted = true;

        // Default "speed" values:
        public static double defaultLowerMotorInSpeed = 0.50;
        public static double defaultLowerMotorOutSpeed = -0.60;
        public static double defaultUpperMotorInSpeed = 0.50;
        public static double defaultUpperMotorOutSpeed = -0.60;
    }

    public static final class IntakeConstants
    {
        // Motor controller is Victor SPX:
        public static final int intakeMotorChannel = 26;

        // Want to use positive percentage values for "in".
        // May need to invert those to get the motor running in the right directions:
        public static boolean isMotorInverted = false;

        // Default "speed" values:
        public static double defaultMotorInSpeed = 1.00;
        public static double defaultMotorOutSpeed = -0.80;

        // These deal with the pneumatics:
        public static final int pneumaticsRetractChannel = 2;
        public static final int pneumaticsExtendChannel = 3;
    }

	public static final class ShooterConstants
	{
        // Motor controllers are Talon SRX:
		public static final int leftMotorChannel = 22;
		public static final int rightMotorChannel = 23;
		public static final boolean isLeftMotorInverted = false;
		public static final boolean isRightMotorInverted = true;

		public static final double defaultLowVelocity = 700;
		public static final double defaultHighVelocity = 1350;
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
        public static final String driveTrainLeftPercentOutputKey = "DT Left % Output";
        public static final String driveTrainRightPercentOutputKey = "DT Right % Output";
        public static final String driveTrainAutoTargetPositionKey = "DT Auto Target Pos";

        public static final String driveTrainPidPKey = "DT PID P";
        public static final String driveTrainPidIKey = "DT PID I";
        public static final String driveTrainPidDKey = "DT PID D";
        public static final String driveTrainPidIZKey = "DT PID IZ";
        public static final String driveTrainPidFFKey = "DT PID FF";
        public static final String driveTrainPidMaxOutputKey = "DT PID Max";
        public static final String driveTrainPidMinOutputKey = "DT PID Min";
        public static final String driveTrainPidTarget = "DT PID Target";
        public static final String driveTrainPidCurrent = "DT PID Current";

		public static final String shooterLowTargetVelocityKey = "Shooter Low RPM";
		public static final String shooterHighTargetVelocityKey = "Shooter High RPM";
		public static final String shooterSetVelocityKey = "Shooter Vel Set";
		public static final String shooterCurrentVelocityLeftKey = "Shooter RPM Left";
		public static final String shooterCurrentVelocityRightKey = "Shooter RPM Rght";
        public static final String shooterRampUpTimeKey = "Shooter Ramp Secs";
        public static final String shooterTargetPercentageKey = "Shooter %";
        public static final String shooterAtSpeedKey = "Shooter at Speed";

        public static final String intakeInPercentageKey = "Intake In %";
        public static final String intakeOutPercentageKey = "Intake Out %";

        public static final String indexerLowerInPercentageKey = "Idx Lower In %";
        public static final String indexerLowerOutPercentageKey = "Idx Lower Out %";
        public static final String indexerUpperInPercentageKey = "Idx Upper In %";
        public static final String indexerUpperOutPercentageKey = "Idx Upper Out %";

        public static final String climberHooksExtendSpeedKey = "Climber Hooks Extend %";
        public static final String climberHooksRetractSpeedKey = "Climber Hooks Retract %";
        public static final String climberHooksAtLimit = "Climber Hooks Limit";
    }
}
