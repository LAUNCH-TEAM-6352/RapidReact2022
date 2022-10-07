// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.SmartDashboardEx;
import frc.robot.Constants.DashboardConstants;

public class DriveTrain extends SubsystemBase
{
    // Lists of motors on left and right side of drive train:
    private final List<CANSparkMax> leftMotors = new ArrayList<CANSparkMax>();
    private final List<CANSparkMax> rightMotors = new ArrayList<CANSparkMax>();

    // A map of motors:
    private final Map<Integer, CANSparkMax> motors = new HashMap<Integer, CANSparkMax>();

    // The following used diring PID control:
    private CANSparkMax pidLeader = null;
    private double targetPosition;

    // The current open loop ramp rate:
    private double openLoopRampRate = DriveTrainConstants.defaultOpenLoopRampRate;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain()
    {
        // Construct the motors on the left side of the drive train:
        for (int channel : DriveTrainConstants.leftMotorChannels)
        {
            var motor = new CANSparkMax(channel, MotorType.kBrushless);
            setCommonConfig(motor);
            motor.setInverted(DriveTrainConstants.isLeftMotorInverted);
            leftMotors.add(motor);
            motors.put(Integer.valueOf(channel), motor);
        }

        // Construct the motors on the right side of the drive train:
        for (int channel : DriveTrainConstants.rightMotorChannels)
        {
            var motor = new CANSparkMax(channel, MotorType.kBrushless);
            setCommonConfig(motor);
            motor.setInverted(DriveTrainConstants.areRightMotorsInverted);
            rightMotors.add(motor);
            motors.put(Integer.valueOf(channel), motor);
        }
    }

    /**
     * Sets up PID controller and leader and followers for PID control.
     */
    public void configureForPidControl()
    {
        // set up leader and followers:
        pidLeader = null;
        leftMotors.forEach((motor) ->
        {
            // Make the first left motor the leader for PID:
            if (pidLeader == null)
            {
                pidLeader = motor;
            }
            else
            {
                motor.follow(pidLeader);
            }
        });

        rightMotors.forEach((motor) ->
        {
            motor.follow(pidLeader, DriveTrainConstants.isLeftMotorInverted != DriveTrainConstants.areRightMotorsInverted);
        });

        // Set closed loop ramp rate on the leader:
        pidLeader.setClosedLoopRampRate(
            SmartDashboardEx.getNumber(DashboardConstants.driveTrainClosedLoopRampRateKey, DriveTrainConstants.defaultClosedLoopRampRate));
        
        // Set PID controller parameters on the leader:
        var pidController = pidLeader.getPIDController();
        pidController.setP(SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidPKey, DriveTrainConstants.defaultPidP));
        pidController.setI(SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidIKey, DriveTrainConstants.defaultPidI));
        pidController.setD(SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidDKey, DriveTrainConstants.defaultPidD));
        pidController.setIZone(SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidIZKey, DriveTrainConstants.defaultPidIZ));
        pidController.setFF(SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidFFKey, DriveTrainConstants.defaultPidFF));
        pidController.setOutputRange(
            SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidMinOutputKey, DriveTrainConstants.defaultPidMinOutput),
            SmartDashboardEx.getNumber(DashboardConstants.driveTrainPidMaxOutputKey, DriveTrainConstants.defaultPidMaxOutput));
    }

    /**
     * Make sure motor controllers are configured for driver control:
     */
    public void configureForDriverControl()
    {
        leftMotors.forEach((motor) -> motor.follow(ExternalFollower.kFollowerDisabled, 0));
        rightMotors.forEach((motor) -> motor.follow(ExternalFollower.kFollowerDisabled, 0));
        openLoopRampRate = SmartDashboardEx.getNumber(DashboardConstants.driveTrainOpenLoopRampRateKey, DriveTrainConstants.defaultOpenLoopRampRate);
        setOpenLoopRampRate(openLoopRampRate);
    }

    /**
     * Using a PID controller, drives the robot to a specific position,
     * where position is expresed in encoder rotations.
     * 
     * @param position encoder counts
     */
    public void driveToPosition(double position)
    {
        targetPosition = position;
        pidLeader.getPIDController().setReference(position, ControlType.kPosition);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidTarget, position);
    }

    /**
     * Determines if PID controller is at target position.
     * 
     * @return true if at the target position
     */
    public boolean isAtTargetPosition()
    {
        return Math.abs(pidLeader.getEncoder().getPosition() - targetPosition) < DriveTrainConstants.positionTolerance;
    }
        
    /**
     * Team Caution style drive using input from two joysticks, left and right.
     * 
     * @param leftStick
     * @param rightStick
     */
    public void driveCaution(Joystick leftStick, Joystick rightStick)
    {
        setTunedOutputs(leftStick.getY() - rightStick.getX(), leftStick.getY() + rightStick.getX());
    }

    /**
     * Team Caution style drive using input from the joysticks on one Xbox controller.
     * 
     * @param controller
     */
    public void driveCaution(XboxController controller)
    {
        setTunedOutputs(controller.getLeftY() - controller.getRightX(), controller.getLeftY() + controller.getRightX());
    }

    /**
     * Squares inputs for more sensitivity.
     * 
     * @param left
     * @param right
     */
    public void setTunedOutputs(double left, double right)
    {
        left = MathUtil.clamp(left, -1.0, +1.0);
        right = MathUtil.clamp(right, -1.0, +1.0);

        double leftOut = Math.copySign(left * left, left);
        double rightOut = Math.copySign(right * right, right);
        
        setPercentage(leftMotors, leftOut);
        setPercentage(rightMotors, rightOut);

        //SmartDashboardEx.putNumber(DashboardConstants.driveTrainLeftPercentOutputKey, leftOut);
        //SmartDashboardEx.putNumber(DashboardConstants.driveTrainRightPercentOutputKey, rightOut);
    }

    /**
     * Sets un-tuned motor percentages.
     * 
     * @param percentage
     */
    public void setRawMotorOutputs(double percentage)
    {
        percentage = MathUtil.clamp(percentage, -1.0, +1.0);

        setPercentage(leftMotors, percentage);
        setPercentage(rightMotors, percentage);

        SmartDashboardEx.putNumber(DashboardConstants.driveTrainLeftPercentOutputKey, percentage);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainRightPercentOutputKey, percentage);
    }

	/**
	 * Stop the drive.
	 */
	public void stop()
	{
        leftMotors.forEach((motor) -> motor.stopMotor());
        rightMotors.forEach((motor) -> motor.stopMotor());

        SmartDashboardEx.putNumber(DashboardConstants.driveTrainLeftPercentOutputKey, 0);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainRightPercentOutputKey, 0);
	}

    /**
     * Runs the motor at the specified channel at the specified percentage.
     * 
     * @param channel
     * @param percentage
     */
    public void set(int channel, double percentage)
    {
        var motor = motors.get(Integer.valueOf(channel));
        if (motor != null)
        {
            motor.set(percentage);
        }
    }

    /**
     * Stops the motor at the speficied channel.
     * 
     * @param channel
     */
    public void stop(int channel)
    {
        var motor = motors.get(Integer.valueOf(channel));
        if (motor != null)
        {
            motor.stopMotor();
        }
    }

    /**
     * Resets encoder position on all motors.
     */
    public void resetPosition()
    {
        leftMotors.forEach((motor) -> motor.getEncoder().setPosition(0));
        rightMotors.forEach((motor) -> motor.getEncoder().setPosition(0));
    }

    @Override
    public void periodic()
    {
        if (!leftMotors.isEmpty())
        {
            SmartDashboardEx.putNumber(DashboardConstants.driveTrainLeftPositionKey, leftMotors.get(0).getEncoder().getPosition());
            // SmartDashboardEx.putNumber("DT Left Applied", leftMotors.get(0).getAppliedOutput());
        }

        if (!rightMotors.isEmpty())
        {
            SmartDashboardEx.putNumber(DashboardConstants.driveTrainRightPositionKey, rightMotors.get(0).getEncoder().getPosition());
            // SmartDashboardEx.putNumber("DT Right Applied", rightMotors.get(0).getAppliedOutput());
        }

        // See if the open loop ramp rate has been changed on the Smart Dashboard:
        // double dashboardRampRate = SmartDashboardEx.getNumber(
        //     DashboardConstants.driveTrainOpenLoopRampRateKey, DriveTrainConstants.defaultOpenLoopRampRate);
        // if (dashboardRampRate != openLoopRampRate)
        // {
        //     openLoopRampRate = dashboardRampRate;
        //     setOpenLoopRampRate(openLoopRampRate);
        // }
    }

    /**
     * Sets configuration values common to all motors.
     * 
     * @param motor
     */
    private void setCommonConfig(CANSparkMax motor)
    {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setIdleMode(DriveTrainConstants.idleMode);
    }

    /**
     * Sets the open loop ramp rate on all motors.
     */
    private void setOpenLoopRampRate(double rampRate)
    {
        leftMotors.forEach((motor) -> motor.setOpenLoopRampRate(rampRate));
        rightMotors.forEach((motor) -> motor.setOpenLoopRampRate(rampRate));
    }

    /**
     * Sets the speeds of the specified motors to the specified percentage.
     * 
     * @param motors The motors to adjust.
     * @param percentage The new percentage. 
     */
    private void setPercentage(List<CANSparkMax> motors, double percentage)
    {
        motors.forEach((motor) -> motor.set(percentage));
    }
}
