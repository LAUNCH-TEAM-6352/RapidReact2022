// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ClimberConstants;
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
import frc.robot.commands.MoveClimberHooks;
import frc.robot.commands.RunIndexerLower;
import frc.robot.commands.RunIndexerUpper;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
    private final Optional<Climber> climber;

    // OI devices:
	private final XboxController gamepad;
	private final Joystick leftStick;
	private final Joystick rightStick;

    // This is from the driver station:
    private final String gameData;

    // Used for choosing the autonomous program:
    Optional<SendableChooser<Command>> autonomousChooser = Optional.of(new SendableChooser<>());

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
        //   -c-    Climber
        // 
        gameData = DriverStation.getGameSpecificMessage().toLowerCase();

        // Create OI devices:
		// gamepad = DriverStation.isJoystickConnected(OIConstants.gamepadPort)
        //     ? new XboxController(OIConstants.gamepadPort)
        //     : null;
        // leftStick = DriverStation.isJoystickConnected(OIConstants.leftJoystickPort)
        //     ? new Joystick(OIConstants.leftJoystickPort)
        //     : null;
        // rightStick = DriverStation.isJoystickConnected(OIConstants.rightJoystickPort)
        //     ? new Joystick(OIConstants.rightJoystickPort)
        //     : null;

        gamepad = new XboxController(OIConstants.gamepadPort);
        leftStick = new Joystick(OIConstants.leftJoystickPort);
        rightStick = new Joystick(OIConstants.rightJoystickPort);

        SmartDashboardEx.putBoolean("Gamepad Detected", gamepad != null);
        SmartDashboardEx.putBoolean("Left Joystick Detected", leftStick != null);
        SmartDashboardEx.putBoolean("Right Joystick Detected", rightStick != null);

        // Start capturing video from the USB camera:
        //UsbCamera camera = CameraServer.startAutomaticCapture();
        //camera.setResolution(640, 480);
        //camera.setResolution(480, 360);

        // Create pneumatics compressor:
        compressor = gameData.isBlank() || gameData.contains("-p-") ? Optional.of(new Compressor(PneumaticsConstants.moduleId, PneumaticsConstants.moduleType)) : Optional.empty();

    	// Create subsystems:
		driveTrain = gameData.isBlank() || gameData.contains("-dt-") ? Optional.of(new DriveTrain()) : Optional.empty();

        shooter = gameData.isBlank() || gameData.contains("-s-") ? Optional.of(new Shooter(gamepad)) : Optional.empty();

        indexer = gameData.isBlank() || gameData.contains("-idx-") ? Optional.of(new Indexer()) : Optional.empty();

        intake = gameData.isBlank() || gameData.contains("-int-") ? Optional.of(new Intake(compressor.isPresent())) : Optional.empty();

        climber = gameData.isBlank() || gameData.contains("-c-") ? Optional.of(new Climber(gamepad, compressor.isPresent())) : Optional.empty();

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

        // Intake buttons:
        intake.ifPresent(this::configureButtonBindings);

        // Climber buttons:
        climber.ifPresent(this::configureButtonBindings);
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
                SmartDashboardEx.getNumber(DashboardConstants.shooterLowTargetVelocityKey, 0)),
                shooter));
        
        // Run/stop the shooter at the speed for the high target:
        new JoystickButton(gamepad, Button.kStart.value)
            .whenPressed(new InstantCommand(() -> shooter.toggleVelocity(
                SmartDashboardEx.getNumber(DashboardConstants.shooterHighTargetVelocityKey, 0)),
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

        var leftBumper = new JoystickButton(gamepad, Button.kLeftBumper.value);
        var rightBumper = new JoystickButton(gamepad, Button.kRightBumper.value);

        // The use of the withInterrput() decorator in the next two
        // bindings prevents the lower indexer from being run in
        // opposite directions at the same time.
        leftBumper.whileHeld(
            new RunIndexerLower(indexer, DashboardConstants.indexerLowerInPercentageKey)
            .withInterrupt(rightBumper::get));
        rightBumper.whileHeld(
            new RunIndexerLower(indexer, DashboardConstants.indexerLowerOutPercentageKey)
            .withInterrupt(leftBumper::get));

        new JoystickButton(gamepad, Button.kY.value)
            .whileHeld(new RunIndexerUpper(indexer, DashboardConstants.indexerUpperInPercentageKey));

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
    }

    private void configureButtonBindings(Climber climber)
    {
        if (leftStick == null || rightStick == null)
        {
            return;
        }

        var extendHooks = new JoystickButton(rightStick, 3);
        var retractHooks = new JoystickButton(rightStick, 4);

        // The use of the withInterrput() decorator in the next two
        // bindings prevents the climber hooks from being run in
        // opposite directions at the same time.
        extendHooks.whileHeld(
            new MoveClimberHooks(climber, DashboardConstants.climberHooksExtendSpeedKey)
            .withInterrupt(retractHooks::get));
        retractHooks.whileHeld(
            new MoveClimberHooks(climber, DashboardConstants.climberHooksRetractSpeedKey)
            .withInterrupt(extendHooks::get));

        new JoystickButton(leftStick, 3)
            .whenPressed(new InstantCommand(() -> climber.moveClimberUp(), climber));
        new JoystickButton(leftStick, 4)
            .whenPressed(new InstantCommand(() -> climber.moveClimberDown(), climber));
    }

    private void configureSmartDashboard()
    {
        driveTrain.ifPresent(this::configureSmartDashboard);

        shooter.ifPresent(this::configureSmartDashboard);

        indexer.ifPresent(this::configureSmartDashboard);

        intake.ifPresent(this::configureSmartDashboard);

        climber.ifPresent(this::configureSmartDashboard);

        autonomousChooser.ifPresent(this::configureSmartDashboard);

        SmartDashboardEx.putBoolean(DashboardConstants.throttleDashboardOutput, true);
    }

    /**
     * Adds the drive train related commands to the Smart Dashboard.
     * 
     * @param driveTrain
     */
    private void configureSmartDashboard(DriveTrain driveTrain)
    {
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPercentageKey, DriveTrainConstants.defaultPercentage);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainOpenLoopRampRateKey, DriveTrainConstants.defaultOpenLoopRampRate);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainClosedLoopRampRateKey, DriveTrainConstants.defaultClosedLoopRampRate);

        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidPKey, DriveTrainConstants.defaultPidP);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidIKey, DriveTrainConstants.defaultPidI);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidDKey, DriveTrainConstants.defaultPidD);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidIZKey, DriveTrainConstants.defaultPidIZ);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidFFKey, DriveTrainConstants.defaultPidFF);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidMinOutputKey, DriveTrainConstants.defaultPidMinOutput);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainPidMaxOutputKey, DriveTrainConstants.defaultPidMaxOutput);

        SmartDashboardEx.putNumber(DashboardConstants.driveTrainAutoTargetPositionKey, DriveTrainConstants.defaultAutoTargetPosition);
        SmartDashboardEx.putNumber(DashboardConstants.driveTrainAutoLeaveTarmacPositionKey, DriveTrainConstants.defaultAutoLeaveTarmacPosition);
        
        SmartDashboardEx.putData("Run Drive Train", new StartEndCommand(
            () -> driveTrain.setRawMotorOutputs(
                SmartDashboardEx.getNumber(DashboardConstants.driveTrainPercentageKey, 0)),
            () -> driveTrain.stop(),
            driveTrain
            )
        );

        // The following deal with driving to a specified position:
        SmartDashboardEx.putData("Drive to Target Pos",
            new DriveToRelativePosition(driveTrain, DashboardConstants.driveTrainAutoTargetPositionKey).withTimeout(10));
        SmartDashboardEx.putData("Drive to Leave Tarmac Pos",
            new DriveToRelativePosition(driveTrain, DashboardConstants.driveTrainAutoLeaveTarmacPositionKey).withTimeout(10));
        SmartDashboardEx.putData("Reset DT Pos", new InstantCommand(() -> driveTrain.resetPosition()));

        // The following are to be used to quickly test the individual drive train motors:
        for (int i = 0; i < DriveTrainConstants.motorNames.length; i++)
        {
            var motorName = DriveTrainConstants.motorNames[i];
            SmartDashboardEx.putData("Test " + motorName.abbreviation + " (" + motorName.channel + ")",
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
        SmartDashboardEx.putNumber(DashboardConstants.shooterLowTargetVelocityKey, ShooterConstants.defaultLowVelocity);
        SmartDashboardEx.putNumber(DashboardConstants.shooterHighTargetVelocityKey, ShooterConstants.defaultHighVelocity);
        SmartDashboardEx.putNumber(DashboardConstants.shooterAutoTargetVelocityKey, ShooterConstants.defaultAutoVelocity);
        SmartDashboardEx.putNumber(DashboardConstants.shooterTargetPercentageKey, ShooterConstants.defaultPercentage);
        SmartDashboardEx.putBoolean(DashboardConstants.shooterAtSpeedKey, false);

        SmartDashboardEx.putData("Run Shooter Low RPM", new StartEndCommand(
            () -> shooter.setVelocity(
                SmartDashboardEx.getNumber(DashboardConstants.shooterLowTargetVelocityKey, 0)),
            () -> shooter.stop(),
            shooter
            )
        );

        SmartDashboardEx.putData("Run Shooter High RPM", new StartEndCommand(
            () -> shooter.setVelocity(
                SmartDashboardEx.getNumber(DashboardConstants.shooterHighTargetVelocityKey, 0)),
            () -> shooter.stop(),
            shooter
            )
        );

        SmartDashboardEx.putData("Run Shooter %", new StartEndCommand(
            () -> shooter.setPercentage(
                SmartDashboardEx.getNumber(DashboardConstants.shooterTargetPercentageKey, 0)),
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
        SmartDashboardEx.putNumber(DashboardConstants.indexerLowerInPercentageKey, IndexerConstants.defaultLowerMotorInSpeed);
        SmartDashboardEx.putNumber(DashboardConstants.indexerLowerOutPercentageKey, IndexerConstants.defaultLowerMotorOutSpeed);
        SmartDashboardEx.putNumber(DashboardConstants.indexerUpperInPercentageKey, IndexerConstants.defaultUpperMotorInSpeed);
        SmartDashboardEx.putNumber(DashboardConstants.indexerUpperOutPercentageKey, IndexerConstants.defaultUpperMotorOutSpeed);
    }

    private void configureSmartDashboard(Intake intake)
    {
        SmartDashboardEx.putNumber(DashboardConstants.intakeInPercentageKey, IntakeConstants.defaultMotorInSpeed);
        SmartDashboardEx.putNumber(DashboardConstants.intakeOutPercentageKey, IntakeConstants.defaultMotorOutSpeed);

        SmartDashboardEx.putData("Extend Intake", new InstantCommand(() -> intake.extend(), intake));
        SmartDashboardEx.putData("Retract Intake", new InstantCommand(() -> intake.retract(), intake));
    }

    private void configureSmartDashboard(Climber climber)
    {
        SmartDashboardEx.putNumber(DashboardConstants.climberHooksExtendSpeedKey, ClimberConstants.defaultHookExtendSpeed);
        SmartDashboardEx.putNumber(DashboardConstants.climberHooksRetractSpeedKey, ClimberConstants.defaultHookRetractSpeed);

        SmartDashboardEx.putData("Climber Up", new InstantCommand(() -> climber.moveClimberUp(), climber));
        SmartDashboardEx.putData("Climber Down", new InstantCommand(() -> climber.moveClimberDown(), climber));

        SmartDashboardEx.putData("Extend Climber Hooks", new MoveClimberHooks(climber, DashboardConstants.climberHooksExtendSpeedKey));
        SmartDashboardEx.putData("Retract Climber Hooks", new MoveClimberHooks(climber, DashboardConstants.climberHooksRetractSpeedKey));
    }

    private void configureSmartDashboard(SendableChooser<Command> chooser)
    {
        // Create an inline sequential command that:
        //   1) Drives to the configured position;
        //   2) Starts the shooter motor at high speed;
        //   3) Waits for the shooter motor to get up to speed;
        //   4) Runs the upper indexer to shoot the cargo; and
        //   5) Stops the shooter motor.
        chooser.setDefaultOption("Auto One Cargo",
            new SequentialCommandGroup(
                new DriveToRelativePosition(driveTrain.get(), DashboardConstants.driveTrainAutoTargetPositionKey).withTimeout(10),
                new InstantCommand(() ->
                    shooter.get().setVelocity(SmartDashboardEx.getNumber(DashboardConstants.shooterAutoTargetVelocityKey, ShooterConstants.defaultHighVelocity)),
                    shooter.get()),
                new WaitUntilCommand(() -> shooter.get().isAtTargetVelocity()).withTimeout(3),
                new RunIndexerUpper(indexer.get(), DashboardConstants.indexerUpperInPercentageKey).withTimeout(3),
                new InstantCommand(() -> shooter.get().stop(), shooter.get())
                )
            );

        chooser.addOption("Auto Leave Tarmac",
            new DriveToRelativePosition(driveTrain.get(), DashboardConstants.driveTrainAutoLeaveTarmacPositionKey).withTimeout(10));

        chooser.addOption("Auto Nothing", new InstantCommand(() -> {}));

        SmartDashboardEx.putData(chooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // To prevent crashing, make sure we have all the necessary subsystems:
        return driveTrain.isEmpty() || shooter.isEmpty() || indexer.isEmpty() || autonomousChooser.isEmpty()
            ? null
            : autonomousChooser.get().getSelected();
    }
}
