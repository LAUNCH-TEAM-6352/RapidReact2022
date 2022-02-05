// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticPrototype extends SubsystemBase
{
    Compressor phCompressor;
    Solenoid exampleSinglePH;
    
    /** Creates a new PneumaticPrototype. */
    public PneumaticPrototype()
    {
        phCompressor = new Compressor(1, PneumaticsConstants.moduleType);
        exampleSinglePH = new Solenoid(PneumaticsConstants.moduleType, 0);
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
