// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveToRelativePosition;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.RunIndexerLower;
import frc.robot.commands.RunIndexerUpper;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
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
    // Pneumatics:
    private final Optional<Compressor> compressor; 

    // Subsystems:
    private final Optional<DriveTrain> driveTrain;
    private final Optional<Shooter> shooter;
    private final Optional<Indexer> indexer;
    private final Optional<Intake> intake;

    // OI devices:
	private final XboxController gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;

    // This is from the driver station:
    private final String gameData;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {     
        // Get the game data message fom the driver station.
        // This message is primarily used during development to
        // construct only certain subsystems.
        // If the merssage is blank (or all whitespace),
        // all subsystems are constructed.
        // Otherwise, OI devices and subsystems are constructed
        // depending upon the substrings found in the message:
        //   -dt-   Drive train
        //   -p-    Pneumatics
        //   -s-    Shooter
        //   -int-  Intake
        //   -idx-  Indexer
        // 
        gameData = DriverStation.getGameSpecificMessage().toLowerCase();

        // Create OI devices:
		gamepad = DriverStation.isJoystickConnected(OIConstants.gamepadPort)
            ? new XboxController(OIConstants.gamepadPort)
            : null;
        leftStick = DriverStation.isJoystickConnected(OIConstants.leftJoystickPort)
            ? new Joystick(OIConstants.leftJoystickPort)
            : null;
        rightStick = DriverStation.isJoystickConnected(OIConstants.rightJoystickPort)
            ? new Joystick(OIConstants.rightJoystickPort)
            : null;

        SmartDashboard.putBoolean("Gamepad Detected", gamepad != null);
        SmartDashboard.putBoolean("Left Joystick Detected", leftStick != null);
        SmartDashboard.putBoolean("Right Joystick Detected", rightStick != null);

        // Start capturing video from the USB camera:
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);


        // Create pneumatics compressor:
        compressor = gameData.isBlank() || gameData.contains("-p-") ? Optional.of(new Compressor(PneumaticsConstants.moduleId, PneumaticsConstants.moduleType)) : Optional.empty();

    	// Create subsystems:
		driveTrain = gameData.isBlank() || gameData.contains("-dt-") ? Optional.of(new DriveTrain()) : Optional.empty();

        shooter = gameData.isBlank() || gameData.contains("-s-") ? Optional.of(new Shooter()) : Optional.empty();

        indexer = gameData.isBlank() || gameData.contains("-idx-") ? Optional.of(new Indexer()) : Optional.empty();

        intake = gameData.isBlank() || gameData.contains("-int-") ? Optional.of(new Intake(compressor.isPresent())) : Optional.empty();

        // Configure default commands:
        configureDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();

        // Configure the Smart Dashboard:
        configureSmartDashboard();
    }

    /**
     * Configures the default commands.
     */
    private void configureDefaultCommands()
    {
        driveTrain.ifPresent((dt) ->
        {
            if (leftStick != null && rightStick != null)
            {
                dt.setDefaultCommand(new DriveWithJoysticks(dt, leftStick, rightStick));
            }
            else if (gamepad != null)
            {
                dt.setDefaultCommand(new DriveWithGamepad(dt, gamepad));
            }
        });
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Shooter buttons:
        shooter.ifPresent(this::configureButtonBindings);

        // Indexer buttons:
        indexer.ifPresent(this::configureButtonBindings);

        //Intake buttons:
        intake.ifPresent(this::configureButtonBindings);
    }

    /**
     * Configures the button bindings for the shooter.
     * 
     * @param shooter
     */
    private void configureButtonBindings(Shooter shooter)
    {
        if (gamepad == null)
        {
            return;
        }

        // Run/stop the shooter at the speed for the low target:
        new JoystickButton(gamepad, Button.kBack.value)
            .whenPressed(new InstantCommand(() -> shooter.toggleVelocity(
                SmartDashboard.getNumber(DashboardConstants.shooterLowTargetVelocityKey, 0)),
                shooter));
        
        // Run/stop the shooter at the speed for the high target:
        new JoystickButton(gamepad, Button.kStart.value)
            .whenPressed(new InstantCommand(() -> shooter.toggleVelocity(
                SmartDashboard.getNumber(DashboardConstants.shooterHighTargetVelocityKey, 0)),
                shooter));
    }

    /**
     * Configure the button bindings for the indexer.
     * 
     * @param indexer
     */
    private void configureButtonBindings(Indexer indexer)
    {
        if (gamepad == null)
        {
            return;
        }

        new JoystickButton(gamepad, Button.kLeftBumper.value)
            .whileHeld(new RunIndexerLower(indexer, DashboardConstants.indexerLowerInPercentageKey));
        new JoystickButton(gamepad, Button.kRightBumper.value)
            .whileHeld(new RunIndexerLower(indexer, DashboardConstants.indexerLowerOutPercentageKey));
        new JoystickButton(gamepad, Button.kY.value)
            .whileHeld(new RunIndexerUpper(indexer, DashboardConstants.indexerUpperInPercentageKey));

        // TODO: Create binding to run upper indexer out?

        // TODO: Create button binding to run lower and upper indexers simultaneously?
    }

    private void configureButtonBindings(Intake intake)
    {
        if (gamepad == null)
        {
            return;
        }

        new JoystickButton(gamepad, Button.kA.value)
            .whileHeld(new RunIntake(intake, DashboardConstants.intakeInPercentageKey));
        new JoystickButton(gamepad, Button.kX.value)
            .whileHeld(new RunIntake(intake, DashboardConstants.intakeOutPercentageKey));

        new JoystickButton(gamepad, Button.kLeftStick.value)
            .whenPressed(new InstantCommand(() -> intake.extend(), intake));
        new JoystickButton(gamepad, Button.kRightStick.value)
            .whenPressed(new InstantCommand(() -> intake.retract(), intake));

        // TODO: Configure button bindings for extending and retracting the intake mechanism.
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);

        shooter.ifPresent(this::configureSmartDashboard);

        indexer.ifPresent(this::configureSmartDashboard);

        intake.ifPresent(this::configureSmartDashboard);
    }

    /**
     * Adds the drive train related commands to the Smart Dashboard.
     * 
     * @param driveTrain
     */
    private void configureSmartDashboard(DriveTrain driveTrain)
    {
        SmartDashboard.putNumber(DashboardConstants.driveTrainPercentageKey, DriveTrainConstants.defaultPercentage);
        SmartDashboard.putNumber(DashboardConstants.driveTrainOpenLoopRampRateKey, DriveTrainConstants.defaultOpenLoopRampRate);
        SmartDashboard.putNumber(DashboardConstants.driveTrainClosedLoopRampRateKey, DriveTrainConstants.defaultClosedLoopRampRate);

        SmartDashboard.putNumber(DashboardConstants.driveTrainPidPKey, DriveTrainConstants.defaultPidP);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidIKey, DriveTrainConstants.defaultPidI);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidDKey, DriveTrainConstants.defaultPidD);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidIZKey, DriveTrainConstants.defaultPidIZ);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidFFKey, DriveTrainConstants.defaultPidFF);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidMinOutputKey, DriveTrainConstants.defaultPidMinOutput);
        SmartDashboard.putNumber(DashboardConstants.driveTrainPidMaxOutputKey, DriveTrainConstants.defaultPidMaxOutput);

        SmartDashboard.putNumber(DashboardConstants.driveTrainAutoTargetPositionKey, DriveTrainConstants.defaultAutoTargetPosition);

        SmartDashboard.putData("Run Drive Train", new StartEndCommand(
            () -> driveTrain.setRawMotorOutputs(
                SmartDashboard.getNumber(DashboardConstants.driveTrainPercentageKey, 0)),
            () -> driveTrain.stop(),
            driveTrain
            )
        );

        // The following deal with driving to a specified position:
        var cmd = new DriveToRelativePosition(driveTrain, DashboardConstants.driveTrainAutoTargetPositionKey).withTimeout(10);
        SmartDashboard.putData("Drive to Position", cmd);
        SmartDashboard.putData("Reset DT Pos", new InstantCommand(() -> driveTrain.resetPosition()));

        // The following are to be used to quickly test the individual drive train motors:
        for (int i = 0; i < DriveTrainConstants.motorNames.length; i++)
        {
            var motorName = DriveTrainConstants.motorNames[i];
            SmartDashboard.putData("Test " + motorName.abbreviation + " (" + motorName.channel + ")",
                new StartEndCommand(
                    () -> driveTrain.set(motorName.channel, 0.25),
                    () -> driveTrain.stop(motorName.channel),
                    driveTrain));
        };
    }

    /**
     * Adds the shooter erlated commands to the Smart Dashboard.
     * 
     * @param shooter
     */
    private void configureSmartDashboard(Shooter shooter)
    {
        SmartDashboard.putNumber(DashboardConstants.shooterLowTargetVelocityKey, ShooterConstants.defaultLowVelocity);
        SmartDashboard.putNumber(DashboardConstants.shooterHighTargetVelocityKey, ShooterConstants.defaultHighVelocity);
        SmartDashboard.putNumber(DashboardConstants.shooterTargetPercentageKey, ShooterConstants.defaultPercentage);
        SmartDashboard.putBoolean(DashboardConstants.shooterAtSpeedKey, false);

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

    /**
     * Adds indexer related stuff to the Smart Dashboard.
     * 
     * @parm indexer
     */
    private void configureSmartDashboard(Indexer indexer)
    {
        SmartDashboard.putNumber(DashboardConstants.indexerLowerInPercentageKey, IndexerConstants.defaultLowerMotorInSpeed);
        SmartDashboard.putNumber(DashboardConstants.indexerLowerOutPercentageKey, IndexerConstants.defaultLowerMotorOutSpeed);
        SmartDashboard.putNumber(DashboardConstants.indexerUpperInPercentageKey, IndexerConstants.defaultUpperMotorInSpeed);
        SmartDashboard.putNumber(DashboardConstants.indexerUpperOutPercentageKey, IndexerConstants.defaultUpperMotorOutSpeed);
    }

    private void configureSmartDashboard(Intake intake)
    {
        SmartDashboard.putNumber(DashboardConstants.intakeInPercentageKey, IntakeConstants.defaultMotorInSpeed);
        SmartDashboard.putNumber(DashboardConstants.intakeOutPercentageKey, IntakeConstants.defaultMotorOutSpeed);

        SmartDashboard.putData("Extend Intake", new InstantCommand(() -> intake.extend(), intake));
        SmartDashboard.putData("Retract Intake", new InstantCommand(() -> intake.retract(), intake));
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
