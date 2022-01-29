// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;

public class DriveTrain extends SubsystemBase
{
    private final List<CANSparkMax> leftMotors;
    private final List<CANSparkMax> rightMotors;

    /** Creates a new DriveTrain. */
    public DriveTrain()
    {
        leftMotors = new ArrayList<CANSparkMax>();
        rightMotors = new ArrayList<CANSparkMax>();

        for (int i = 0; i < Constants.DriveTrainConstants.leftMotorChannels.length; i++)
        {
            var motor = new CANSparkMax(Constants.DriveTrainConstants.leftMotorChannels[i], MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setInverted(Constants.DriveTrainConstants.leftMotorsInverted);
            motor.setIdleMode(IdleMode.kBrake);
            if (i > 0)
            {
                motor.follow(leftMotors.get(0));
            }
            leftMotors.add(motor);
        }

        for (int i = 0; i < Constants.DriveTrainConstants.rightMotorChannels.length; i++)
        {
            var motor = new CANSparkMax(Constants.DriveTrainConstants.rightMotorChannels[i], MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setInverted(Constants.DriveTrainConstants.rightMotorsInverted);
            motor.setIdleMode(IdleMode.kBrake);
            if (i > 0)
            {
                motor.follow(rightMotors.get(0));
            }
            rightMotors.add(motor);
        }
    }
        
    /**
     * Team Caution style drive using input from two joysticks, left and right.
     * @param leftStick
     * @param rightStick
     */
    public void driveCaution(Joystick leftStick, Joystick rightStick)
    {
        setTunedMotorOutputs(leftStick.getY() - rightStick.getX(), leftStick.getY() + rightStick.getX());
    }

    /**
     * Team Caution style drive using input from one Xbox controller.
     * @param controller
     */
    public void driveCaution(XboxController controller)
    {
        setTunedMotorOutputs(controller.getLeftY() - controller.getRightX(), controller.getLeftY() + controller.getRightX());
    }

    /**
     * Squares inputs for more sensitivity.
     * 
     * @param left
     * @param right
     */
    public void setTunedMotorOutputs(double left, double right)
    {
        left = MathUtil.clamp(left, -1.0, +1.0);
        right = MathUtil.clamp(right, -1.0, +1.0);
        
        leftMotors.get(0).set(Math.copySign(left * left, left));
        rightMotors.get(0).set(Math.copySign(right * right, right));
    }

    /**
     * Sets un-tuned motor percentages.
     * 
     * @param percentage
     */
    public void setRawMotorOutputs(double percentage)
    {
        percentage = MathUtil.clamp(percentage, -1.0, +1.0);
        
        leftMotors.get(0).set(percentage);
        rightMotors.get(0).set(percentage);
    }

	/**
	 * Stop the drive;
	 */
	public void stop()
	{
		leftMotors.get(0).set(0);
		rightMotors.get(0).set(0);
	}

    public void resetPosition()
    {
        leftMotors.get(0).getEncoder().setPosition(0.0);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber(DashboardConstants.driveTrainPositionKey, leftMotors.get(0).getEncoder().getPosition());
    }
}
