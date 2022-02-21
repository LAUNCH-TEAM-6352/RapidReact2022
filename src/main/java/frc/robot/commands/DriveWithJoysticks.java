// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase
{
    private final DriveTrain driveTrain;
	private Joystick leftStick;
	private Joystick rightStick;

    /** Creates a new DriveWithJoysticks. */
    public DriveWithJoysticks(DriveTrain driveTrain, Joystick leftStick, Joystick rightStick)
    {
        this.driveTrain = driveTrain;
		this.leftStick = leftStick;
		this.rightStick = rightStick;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        driveTrain.configureForDriverControl();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        driveTrain.driveCaution(leftStick, rightStick);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
