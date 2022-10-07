/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SmartDashboardEx;
import frc.robot.subsystems.Indexer;

/**
 * A command for running the upper indexer.
 * <p>
 * This command does not require the {@link Indexer} subsystem to allow
 * it to run simultaneoulsy with the {@link RunIndexerLower} command.
 */
public class RunIndexerUpper extends CommandBase
{
	private final Indexer indexer;
	private String key = null;
	private double percentage;

	private RunIndexerUpper(Indexer indexer)
	{
		this.indexer = indexer;
	}

	public RunIndexerUpper(Indexer indexer, String key)
	{
		this(indexer);
		this.key = key;
	}

	public RunIndexerUpper(Indexer indexer, double percentage)
	{
		this(indexer);
		this.percentage = percentage;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
		if (key != null)
		{
			percentage = SmartDashboardEx.getNumber(key, 0.0);
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		indexer.setUpper(percentage);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		indexer.stopUpper();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
