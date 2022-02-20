/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * A command for running the lower intake.
 */
public class RunIntake extends CommandBase
{
	private final Intake intake;
	private String key = null;
	private double percentage;

	private RunIntake(Intake intake)
	{
		this.intake = intake;
		addRequirements(intake);
	}

	public RunIntake(Intake intake, String key)
	{
		this(intake);
		this.key = key;
	}

	public RunIntake(Intake intake, double percentage)
	{
		this(intake);
		this.percentage = percentage;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
		if (key != null)
		{
			percentage = SmartDashboard.getNumber(key, 0.0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		intake.set(percentage);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		intake.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
