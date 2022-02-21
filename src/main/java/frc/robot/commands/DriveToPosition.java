// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToPosition extends CommandBase
{
    private final DriveTrain driveTrain;
	private String key = null;
	private double position;

    private DriveToPosition(DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);

        // Forces command to stop after 5 seconds:
        withTimeout(5);
    }

	public DriveToPosition(DriveTrain driveTrain, String key)
	{
		this(driveTrain);
		this.key = key;
	}

	public DriveToPosition(DriveTrain driveTrain, double position)
	{
		this(driveTrain);
		this.position = position;
	}

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (key != null)
        {
            position = SmartDashboard.getNumber(key, 0.0);
        }
        driveTrain.configureForPidControl();
        driveTrain.resetPosition();
        driveTrain.driveToPosition(position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
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
        return driveTrain.isAtTargetPosition();
    }
}
