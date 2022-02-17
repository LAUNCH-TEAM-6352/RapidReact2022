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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterMotorConstants;

public class Shooter extends SubsystemBase
{
	private TalonSRX leftMotor = new TalonSRX(ShooterConstants.leftMotorChannel);
	private TalonSRX rightMotor = new TalonSRX(ShooterConstants.rightMotorChannel);

    // Indicates if the shooter is running:
    private boolean isRunning = false;

    // Values to determine how long it takes the shooter motor to ramp up to the target velocity:
    // The FPGA microsecond timer:
    private long startTime;
    // The target motor velocity:
    private double targetVelocity;
    // The motor velocity when the operation started:
    private double lastVelocity;
    // Indicates if we have started timeing an operatrion:
    private boolean isTimingStarted = false;

	/**
	 * Creates an instance.
	 */
	public Shooter()
	{
		leftMotor.setInverted(ShooterConstants.isLeftMotorInverted);
		rightMotor.setInverted(ShooterConstants.isRightMotorInverted);

		rightMotor.set(ControlMode.Follower, leftMotor.getDeviceID());

		for (TalonSRX motor : new TalonSRX[] { leftMotor, rightMotor})
		{
			motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
			motor.configAllowableClosedloopError(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidAllowableError,
					ShooterMotorConstants.pidTimeoutMs);
			motor.configClosedLoopPeakOutput(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidPeakOutput,
					ShooterMotorConstants.pidTimeoutMs);
			motor.configClosedLoopPeriod(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidLoopPeriodMs,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kP(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidP,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kI(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidI,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kD(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidD,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_kF(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidFF,
					ShooterMotorConstants.pidTimeoutMs);
			motor.config_IntegralZone(ShooterMotorConstants.profileSlot, ShooterMotorConstants.pidIZ,
					ShooterMotorConstants.pidTimeoutMs);
			motor.selectProfileSlot(ShooterMotorConstants.profileSlot, ShooterMotorConstants.primaryClosedLoop);
			motor.setSensorPhase(ShooterMotorConstants.phase);
            motor.setNeutralMode(ShooterMotorConstants.neutralMode);
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

	/***
	 * Sets the shooter motor speeds in velocity (RPM).
	 */
	public void setVelocity(double velocity)
	{
		// Velocity is measured in encoder units per 100 ms.
		// 600.0 is the number of 100ms per minute.
		var unitsPer100Ms =
			velocity * ShooterMotorConstants.countsPerRevolution * ShooterMotorConstants.ticksPerCount / 600.0;
		SmartDashboard.putNumber(DashboardConstants.shooterSetVelocityKey, unitsPer100Ms);
        targetVelocity = velocity;
        lastVelocity = 0;
        startTime = RobotController.getFPGATime();
		leftMotor.set(ControlMode.Velocity, unitsPer100Ms);
        isTimingStarted = true;
        isRunning = true;
	}

	/***
	 * Sets the shooter motor speeds in percentage.
	 */
	public void setPercentage(double percentage)
	{
		leftMotor.set(ControlMode.PercentOutput, percentage);
	}

	/***
	 * Stops the shooter motors.
	 */
	public void stop()
	{
		setPercentage(0);
        isRunning = false;
	}

	@Override
	public void periodic()
	{
		// Velocity from motor controller is units per 100ms where there are
		// countsPerRevolution * ticksPerCount units per revolution.
		// 600.0 is the number of 100ms per minute.
        double leftVelocity = 600.0 * leftMotor.getSelectedSensorVelocity() / ShooterMotorConstants.countsPerRevolution / ShooterMotorConstants.ticksPerCount;
        double rightVelocity = 600.0 * rightMotor.getSelectedSensorVelocity() / ShooterMotorConstants.countsPerRevolution / ShooterMotorConstants.ticksPerCount;

		SmartDashboard.putNumber(DashboardConstants.shooterCurrentVelocityLeftKey, leftVelocity);	
		SmartDashboard.putNumber(DashboardConstants.shooterCurrentVelocityRightKey,  rightVelocity);

        if (isTimingStarted)
        {
            if (Math.abs(leftVelocity - targetVelocity) < 10 && Math.abs(leftVelocity - lastVelocity) < 10)
            {
                isTimingStarted = false;
                long stopTime = RobotController.getFPGATime();
                SmartDashboard.putNumber(DashboardConstants.shooterRampUpTimeKey, (stopTime - startTime) / 1000000.0);
            }
            else
            {
                lastVelocity = leftVelocity;
            }
        }
	}
}
