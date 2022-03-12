// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.PneumaticsConstants;

/**
 * Implements the climber.
 */
public class Climber extends SubsystemBase
{
    // For rumbling when limits reached:
    private final XboxController gamepad;

    // Motors for extending and retracting the grab hooks:
    private final VictorSPX leftMotor = new VictorSPX(ClimberConstants.leftMotorChannel);
    private final VictorSPX rightMotor = new VictorSPX(ClimberConstants.rightMotorChannel);

    // Digital I/O channels for extension/retraction limit switches:
    private final DigitalInput hookExtendLimit = new DigitalInput(ClimberConstants.extendLimitChannel);
    private final DigitalInput hookRetractLimit = new DigitalInput(ClimberConstants.retractLimitChannel);

    // The solenoid for moving the climber mechanism up and down:
    private DoubleSolenoid solenoid;

    /**
     * Creates a new Climber.
     */
    public Climber(XboxController gamepad, boolean isPneumaticsPresent)
    {
        this.gamepad = gamepad;

        // Initialize the motor controllers:
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
        leftMotor.setInverted(ClimberConstants.isLeftMotorInverted);
        rightMotor.setInverted(ClimberConstants.isRightMotorInverted);
		rightMotor.set(ControlMode.Follower, leftMotor.getDeviceID());

        // Conditionally create the solenoid for moving the climber up and down:
        solenoid = isPneumaticsPresent
            ? new DoubleSolenoid(PneumaticsConstants.moduleType, ClimberConstants.pneumaticsUpChannel, ClimberConstants.pneumaticsDownChannel)
            : null;
    }

    /**
     * Runs the hook moors at the spexcified percentage speed.
     * 
     * @param speed
     */
    public void setHooks(double speed)
    {
        // If trying to move past a limit, stop the motors and rumble the gamepad:
        if ((speed > 0 && isAtHookExtendLimit()) || (speed < 0 && isAtHookRetractLimit()))
        {
			speed = 0;
			setGamepadRumble(1);
		}
		else
		{
			setGamepadRumble(0);
		}

        leftMotor.set(ControlMode.PercentOutput, speed);
    }

	/**
	 * Stops the hook motors.
	 */
	public void stopHooks()
	{
		setHooks(0);
	}

    /**
     * Returns <code>true</code> if the hooks are at the extend limit.
     */
    public boolean isAtHookExtendLimit()
    {
        return !hookExtendLimit.get();
    }

    /**
     * Returns <code>true</code> if the hooks are at the retract limit.
     */
    public boolean isAtHookRetractLimit()
    {
        return !hookRetractLimit.get();
    }

    /**
     * Moves the climber mechanism up.
     */
    public void moveClimberUp()
    {
        solenoid.set(Value.kForward);
    }

    /**
     * Moves the climber mechanism down.
     */
    public void moveClimberDown()
    {
        solenoid.set(Value.kReverse);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean(DashboardConstants.climberHooksAtLimit, !(isAtHookExtendLimit() || isAtHookRetractLimit()));
    }

    private void setGamepadRumble(double value)
    {
        if (gamepad != null)
        {
            gamepad.setRumble(RumbleType.kLeftRumble, value);
        }
    }
}
