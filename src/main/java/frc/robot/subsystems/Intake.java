/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;

/**
 * Add your docs here.
 */
public class Intake extends SubsystemBase
{
	private final VictorSPX motor = new VictorSPX(IntakeConstants.intakeMotorChannel);

    private final DoubleSolenoid solenoid;

	public Intake(boolean isPneumaticsPresent)
	{
		motor.setInverted(IntakeConstants.isMotorInverted);

        solenoid = isPneumaticsPresent
            ? new DoubleSolenoid(PneumaticsConstants.moduleType, IntakeConstants.pneumaticsExtendChannel, IntakeConstants.pneumaticsRetractChannel)
            : null;
    }

    /**
     * Extends the intake mechanism.
     */
    public void extend()
    {
        solenoid.set(Value.kForward);
    }

    /**
     * Retracts the intake mechanism.
     */
    public void retract()
    {
        solenoid.set(Value.kReverse);
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
