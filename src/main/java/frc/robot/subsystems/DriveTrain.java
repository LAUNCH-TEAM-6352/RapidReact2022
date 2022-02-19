// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DashboardConstants;

public class DriveTrain extends SubsystemBase
{
    // Lists of motors on left and right side of drive train:
    private final List<CANSparkMax> leftMotors = new ArrayList<CANSparkMax>();
    private final List<CANSparkMax> rightMotors = new ArrayList<CANSparkMax>();

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
            motor.getEncoder().setInverted(DriveTrainConstants.isLeftEncoderInverted);
            leftMotors.add(motor);
        }

        // Construct the motors on the right side of the drive train:
        for (int channel : DriveTrainConstants.rightMotorChannels)
        {
            var motor = new CANSparkMax(channel, MotorType.kBrushless);
            setCommonConfig(motor);
            motor.setInverted(DriveTrainConstants.areRightMotorsInverted);
            motor.getEncoder().setInverted(DriveTrainConstants.isRightEncoderInverted);
            rightMotors.add(motor);
        }
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

        SmartDashboard.putNumber(DashboardConstants.driveTrainLeftPercentOutput, leftOut);
        SmartDashboard.putNumber(DashboardConstants.driveTrainRightPercentOutput, rightOut);
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

        SmartDashboard.putNumber(DashboardConstants.driveTrainLeftPercentOutput, percentage);
        SmartDashboard.putNumber(DashboardConstants.driveTrainRightPercentOutput, percentage);
    }

	/**
	 * Stop the drive.
	 */
	public void stop()
	{
        setPercentage(leftMotors, 0);
        setPercentage(rightMotors, 0);

        SmartDashboard.putNumber(DashboardConstants.driveTrainLeftPercentOutput, 0);
        SmartDashboard.putNumber(DashboardConstants.driveTrainRightPercentOutput, 0);
	}

    public void resetPosition()
    {
        leftMotors.get(0).getEncoder().setPosition(0.0);
    }

    @Override
    public void periodic()
    {
        if (!leftMotors.isEmpty())
        {
            SmartDashboard.putNumber(DashboardConstants.driveTrainLeftPositionKey, leftMotors.get(0).getEncoder().getPosition());
        }

        if (!rightMotors.isEmpty())
        {
            SmartDashboard.putNumber(DashboardConstants.driveTrainRightPositionKey, rightMotors.get(0).getEncoder().getPosition());
        }
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
        motor.setOpenLoopRampRate(SmartDashboard.getNumber(DashboardConstants.driveTrainOpenLoopRampRateKey, 0.0));
        motor.setClosedLoopRampRate(SmartDashboard.getNumber(DashboardConstants.driveTrainClosedLoopRampRateKey, 0.0));

        // Set PID controller parameters:
        SparkMaxPIDController pidController = motor.getPIDController();
        pidController.setP(DriveTrainConstants.pidP);
        pidController.setI(DriveTrainConstants.pidI);
        pidController.setD(DriveTrainConstants.pidD);
        pidController.setIZone(DriveTrainConstants.pidIZ);
        pidController.setOutputRange(DriveTrainConstants.pidMinOutput, DriveTrainConstants.pidMaxOutput);
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
