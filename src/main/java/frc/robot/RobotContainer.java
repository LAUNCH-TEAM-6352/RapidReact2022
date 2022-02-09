// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithGameController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PneumaticPrototype;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems:
    private final DriveTrain driveTrain;
    private final PneumaticPrototype pneumaticPrototype;

    // OI devices:
	private final XboxController gameController;

	private Joystick leftStick = null;
	private Joystick rightStick = null;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {        
        // Create OI devices:
		gameController = new XboxController(OIConstants.xboxControllerPort);
        
		leftStick = new Joystick(OIConstants.leftJoystickPort);
		rightStick = new Joystick(OIConstants.rightJoystickPort);
    

    	// Create subsystems:
		driveTrain = new DriveTrain();
        //pneumaticPrototype = null;
        pneumaticPrototype = new PneumaticPrototype();

        // Configure default commands:
        driveTrain.setDefaultCommand(new DriveWithGameController(driveTrain, gameController));
        //driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain, leftStick, rightStick));

        // Configure the button bindings
        configureButtonBindings();

        // Initialize the Smart Dashboard:
        initSmartDashboard();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        SmartDashboard.putData("Run Drive Train", new StartEndCommand(
			() -> driveTrain.setRawMotorOutputs(
				SmartDashboard.getNumber(DashboardConstants.driveTrainPercentageKey, 0)),
			() -> driveTrain.stop(),
			driveTrain
			)
        );

        SmartDashboard.putData("Reset Drive Train Pos", new InstantCommand(() -> driveTrain.resetPosition()));
        SmartDashboard.putData("Solenoid Off", new InstantCommand(() -> pneumaticPrototype.setOff()));
        SmartDashboard.putData("Solenoid Forward", new InstantCommand(() -> pneumaticPrototype.setForward()));
        SmartDashboard.putData("Solenoid Reverse", new InstantCommand(() -> pneumaticPrototype.setReverse()));
    }

    private void initSmartDashboard()
    {
        SmartDashboard.putNumber(DashboardConstants.driveTrainPercentageKey, DriveTrainConstants.defaultPercentage);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
