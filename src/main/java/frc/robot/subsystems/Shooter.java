/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartDashboardEx;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase
{
	private final TalonSRX leftMotor = new TalonSRX(ShooterConstants.leftMotorChannel);
	private final TalonSRX rightMotor = new TalonSRX(ShooterConstants.rightMotorChannel);

    // Indicates if the shooter is running:
    private boolean isRunning = false;
    private boolean isAtTargetVelocity = false;

    // Values to determine how long it takes the shooter motor to ramp up to the target velocity:
    // The FPGA microsecond timer:
    private long startTime;
    // The target motor velocity:
    private double targetVelocity;
    // The motor velocity when the operation started:
    private double lastVelocity;
    // Indicates if we have started timeing an operatrion:
    private boolean isTimingStarted = false;

    private final XboxController gamepad;

	/**
	 * Creates an instance.
	 */
	public Shooter(XboxController gamepad)
	{
        this.gamepad = gamepad;

		leftMotor.setInverted(ShooterConstants.isLeftMotorInverted);
		rightMotor.setInverted(ShooterConstants.isRightMotorInverted);

		rightMotor.set(ControlMode.Follower, leftMotor.getDeviceID());

		for (TalonSRX motor : new TalonSRX[] { leftMotor, rightMotor})
		{
			motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
			motor.configAllowableClosedloopError(ShooterConstants.pidProfileSlot, ShooterConstants.pidAllowableError,
					ShooterConstants.pidTimeoutMs);
			motor.configClosedLoopPeakOutput(ShooterConstants.pidProfileSlot, ShooterConstants.pidPeakOutput,
					ShooterConstants.pidTimeoutMs);
			motor.configClosedLoopPeriod(ShooterConstants.pidProfileSlot, ShooterConstants.pidLoopPeriodMs,
					ShooterConstants.pidTimeoutMs);
			motor.config_kP(ShooterConstants.pidProfileSlot, ShooterConstants.pidP,
					ShooterConstants.pidTimeoutMs);
			motor.config_kI(ShooterConstants.pidProfileSlot, ShooterConstants.pidI,
					ShooterConstants.pidTimeoutMs);
			motor.config_kD(ShooterConstants.pidProfileSlot, ShooterConstants.pidD,
					ShooterConstants.pidTimeoutMs);
			motor.config_kF(ShooterConstants.pidProfileSlot, ShooterConstants.pidFF,
					ShooterConstants.pidTimeoutMs);
			motor.config_IntegralZone(ShooterConstants.pidProfileSlot, ShooterConstants.pidIZ,
					ShooterConstants.pidTimeoutMs);
			motor.selectProfileSlot(ShooterConstants.pidProfileSlot, ShooterConstants.primaryClosedLoop);
			motor.setSensorPhase(ShooterConstants.isSensorPhaseInverted);
            motor.setNeutralMode(ShooterConstants.neutralMode);
		}
	}

    /**
     * Toggles the shooter between running at the indicates RPM or stopped.
     * 
     * @param velocity The RPM at the encoder.
     */
    public void toggleVelocity(double velocity)
    {
        if (isRunning)
        {
            stop();
        }
        else
        {
            setVelocity(velocity);
        }
    }

	/**
	 * Sets the shooter motor speeds in velocity (RPM).
	 */
	public void setVelocity(double velocity)
	{
		// Velocity is measured in encoder units per 100 ms.
		// 600.0 is the number of 100ms per minute.
		var unitsPer100Ms =
			velocity * ShooterConstants.countsPerRevolution * ShooterConstants.ticksPerCount / 600.0;
		SmartDashboardEx.putNumber(DashboardConstants.shooterSetVelocityKey, unitsPer100Ms);
        targetVelocity = velocity;
        lastVelocity = 0;
        startTime = RobotController.getFPGATime();
		leftMotor.set(ControlMode.Velocity, unitsPer100Ms);
        isTimingStarted = true;
        isRunning = true;
        isAtTargetVelocity = false;
	}

    /**
     * Returns <code>true</code> if the shooter is at the set target velocity.
     */
    public boolean isAtTargetVelocity()
    {
        return isAtTargetVelocity;
    }

	/***
	 * Sets the shooter motor speeds in percentage.
	 */
	public void setPercentage(double percentage)
	{
		leftMotor.set(ControlMode.PercentOutput, percentage);
	}

	/**
	 * Stops the shooter motors.
	 */
	public void stop()
	{
		setPercentage(0);
        isRunning = false;
        isAtTargetVelocity = false;
        SmartDashboardEx.putBoolean(DashboardConstants.shooterAtSpeedKey, false);
        setGamepadRumble(0);

	}

	@Override
	public void periodic()
	{

		// SmartDashboardEx.putNumber(DashboardConstants.shooterCurrentVelocityLeftKey, leftVelocity);	
		// SmartDashboardEx.putNumber(DashboardConstants.shooterCurrentVelocityRightKey,  rightVelocity);

        if (isTimingStarted)
        {
            // Velocity from motor controller is units per 100ms where there are
            // countsPerRevolution * ticksPerCount units per revolution.
            // 600.0 is the number of 100ms per minute.
            double leftVelocity = 600.0 * leftMotor.getSelectedSensorVelocity() / ShooterConstants.countsPerRevolution / ShooterConstants.ticksPerCount;
            //double rightVelocity = 600.0 * rightMotor.getSelectedSensorVelocity() / ShooterConstants.countsPerRevolution / ShooterConstants.ticksPerCount;

            if (Math.abs(leftVelocity - targetVelocity) < 10 && Math.abs(leftVelocity - lastVelocity) < 10)
            {
                isTimingStarted = false;
                long stopTime = RobotController.getFPGATime();
                isAtTargetVelocity = true;
                SmartDashboardEx.putBoolean(DashboardConstants.shooterAtSpeedKey, true);
                setGamepadRumble(1);
                // The FPGA time is in microseconds, so divide by 1000000.0 to get seconds:
                SmartDashboardEx.putNumber(DashboardConstants.shooterRampUpTimeKey, (stopTime - startTime) / 1000000.0);
            }
            else
            {
                lastVelocity = leftVelocity;
            }
        }
	}

    private void setGamepadRumble(double value)
    {
        if (gamepad != null)
        {
            gamepad.setRumble(RumbleType.kLeftRumble, value);
        }
    }
}
