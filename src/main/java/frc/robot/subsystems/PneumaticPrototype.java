// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticPrototype extends SubsystemBase
{
    Compressor phCompressor;
    Solenoid exampleSinglePH;
    
    /** Creates a new PneumaticPrototype. */
    public PneumaticPrototype()
    {
        phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
        exampleSinglePH = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    public void toggleExamplePH()
    {
        exampleSinglePH.toggle();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
