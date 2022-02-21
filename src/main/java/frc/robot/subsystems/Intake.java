/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class Intake extends SubsystemBase
{
	private VictorSPX motor = new VictorSPX(IntakeConstants.intakeMotorChannel);

	public Intake()
	{
		motor.setInverted(IntakeConstants.isMotorInverted);
	}

	public void set(double percentage)
	{
		motor.set(ControlMode.PercentOutput, percentage);
	}

	public void stop()
	{
		set(0);
	}
}