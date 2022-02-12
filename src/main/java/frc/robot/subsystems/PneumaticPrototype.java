// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticPrototype extends SubsystemBase
{
    Compressor phCompressor;
    DoubleSolenoid exampleDouble;
    
    /** Creates a new PneumaticPrototype. */
    public PneumaticPrototype()
    {
        phCompressor = new Compressor(0, PneumaticsConstants.moduleType);
        exampleDouble = new DoubleSolenoid(PneumaticsConstants.moduleType, 0, 1);
    }

    public void setOff()
    {
        exampleDouble.set(DoubleSolenoid.Value.kOff);
    }

    public void setForward()
    {
        exampleDouble.set(DoubleSolenoid.Value.kForward);
    }

    public void setReverse()
    {
        exampleDouble.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
