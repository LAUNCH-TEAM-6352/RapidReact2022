// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * A command for moving the climber hooks.
 * 
 * We do not require the {@link Climber} mechanism to allow
 * simultaneous pneumatics and hooks operations.
 */
public class MoveClimberHooks extends CommandBase
{
    private final Climber climber;
	private String key = null;
	private double percentage;

    /**
     * Creates a new MoveClimberHooks.
     */
    private MoveClimberHooks(Climber climber)
    {
        this.climber = climber;
    }

    public MoveClimberHooks(Climber climber, String key)
    {
        this(climber);
        this.key = key;
    }

    public MoveClimberHooks(Climber climber, double percentage)
    {
        this(climber);
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
        climber.setHooks(percentage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        climber.stopHooks();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
