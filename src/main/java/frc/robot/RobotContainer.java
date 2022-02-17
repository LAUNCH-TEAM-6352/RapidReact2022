// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveWithGameController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PneumaticPrototype;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final Shooter shooter;

    // OI devices:
	private final XboxController gameController;

	private Joystick leftStick = null;
	private Joystick rightStick = null;

    // This is from the driver station:
    private final String gameData;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {     
        // Get the game data message fom the driver station:

        gameData = DriverStation.getGameSpecificMessage().toLowerCase();

        // Create OI devices:
		gameController = new XboxController(OIConstants.xboxControllerPort);
        
        if (gameData.isBlank() || gameData.contains("-dtjs-"))
        {
            leftStick = new Joystick(OIConstants.leftJoystickPort);
            rightStick = new Joystick(OIConstants.rightJoystickPort);
        }

    	// Create subsystems:
		driveTrain = gameData.isBlank() || gameData.contains("-dt-") ? new DriveTrain() : null;

        pneumaticPrototype = gameData.contains("-pp-") ? new PneumaticPrototype() : null;

        shooter = gameData.contains("-s-") ? new Shooter() : null;

        // Configure default commands:
        if (driveTrain != null)
        {
            driveTrain.setDefaultCommand(
                leftStick != null && rightStick != null
                    ? new DriveWithJoysticks(driveTrain, leftStick, rightStick)
                    : new DriveWithGameController(driveTrain, gameController)
                );
        }

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
        // Drive Train buttons:
        if (driveTrain != null)
        {
            SmartDashboard.putData("Run Drive Train", new StartEndCommand(
                () -> driveTrain.setRawMotorOutputs(
                    SmartDashboard.getNumber(DashboardConstants.driveTrainPercentageKey, 0)),
                () -> driveTrain.stop(),
                driveTrain
                )
            );
        }

        // Shooter buttons:
        if (shooter != null)
        {
            // Run/stop the shooter at the speed for the low target:
            new JoystickButton(gameController, Button.kBack.value)
                .whenPressed(new InstantCommand(() -> shooter.toggleVelocity(
                    SmartDashboard.getNumber(DashboardConstants.shooterLowTargetVelocityKey, 0)),
                    shooter));
            
            // Run/stop the shooter at the speed for the high target:
            new JoystickButton(gameController, Button.kStart.value)
                .whenPressed(new InstantCommand(() -> shooter.toggleVelocity(
                    SmartDashboard.getNumber(DashboardConstants.shooterHighTargetVelocityKey, 0)),
                    shooter));
        }

        SmartDashboard.putData("Reset Drive Train Pos", new InstantCommand(() -> driveTrain.resetPosition()));
    }

    private void initSmartDashboard()
    {
        SmartDashboard.putNumber(DashboardConstants.driveTrainPercentageKey, DriveTrainConstants.defaultPercentage);
        SmartDashboard.putNumber(DashboardConstants.shooterLowTargetVelocityKey, ShooterConstants.defaultLowVelocity);
        SmartDashboard.putNumber(DashboardConstants.shooterHighTargetVelocityKey, ShooterConstants.defaultHighVelocity);
        SmartDashboard.putNumber(DashboardConstants.shooterTargetPercentageKey, ShooterConstants.defaultPercentage);


        if (pneumaticPrototype != null)
        {
            SmartDashboard.putData("Solenoid Off", new InstantCommand(() -> pneumaticPrototype.setOff()));
            SmartDashboard.putData("Solenoid Forward", new InstantCommand(() -> pneumaticPrototype.setForward()));
            SmartDashboard.putData("Solenoid Reverse", new InstantCommand(() -> pneumaticPrototype.setReverse()));
        }

        if (shooter != null)
        {
            SmartDashboard.putData("Run Shooter Low RPM", new StartEndCommand(
                () -> shooter.setVelocity(
                    SmartDashboard.getNumber(DashboardConstants.shooterLowTargetVelocityKey, 0)),
                () -> shooter.stop(),
                shooter
                )
            );

            SmartDashboard.putData("Run Shooter High RPM", new StartEndCommand(
                () -> shooter.setVelocity(
                    SmartDashboard.getNumber(DashboardConstants.shooterHighTargetVelocityKey, 0)),
                () -> shooter.stop(),
                shooter
                )
            );

            SmartDashboard.putData("Run Shooter %", new StartEndCommand(
                () -> shooter.setPercentage(
                    SmartDashboard.getNumber(DashboardConstants.shooterTargetPercentageKey, 0)),
                () -> shooter.stop(),
                shooter
                )
            );
        }
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
