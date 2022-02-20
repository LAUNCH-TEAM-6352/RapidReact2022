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
import frc.robot.Constants.IndexerConstants;

/**
 * Add your docs here.
 */
public class Indexer extends SubsystemBase
{
	private VictorSPX lowerMotor = new VictorSPX(IndexerConstants.lowerMotorChannel);
	private VictorSPX upperMotor = new VictorSPX(IndexerConstants.upperMotorChannel);

	public Indexer()
	{
		lowerMotor.setInverted(IndexerConstants.isLowerMotorInverted);
		upperMotor.setInverted(IndexerConstants.isUpperMotorInverted);
	}

	public void setLower(double percentage)
	{
		lowerMotor.set(ControlMode.PercentOutput, percentage);
	}

	public void setUpper(double percentage)
	{
		upperMotor.set(ControlMode.PercentOutput, percentage);
	}

	public void stopLower()
	{
		setLower(0);
	}

	public void stopUpper()
	{
		setUpper(0);
	}
}
