// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRack;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.RollerFloor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SubsystemManager.RobotState;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick;
    public final CommandXboxController cojoystick = new CommandXboxController(1);

    public final Drivetrain drivetrain;
    public final Hood hood;
    public final Indexer indexer;
    public final IntakeRack intakeRack;
    public final IntakeRoller intakeRoller;
    public final RollerFloor rollerFloor;
    public final Shooter shooter;

    public final Autoaim autoaim;

    public final SubsystemManager subsystemManager;

    public RobotContainer(
        CommandXboxController joystick,
        Autoaim autoaim,
        SubsystemManager subsystemManager,
        Drivetrain drivetrain, 
        Hood hood, 
        Indexer indexer, 
        IntakeRack intakeRack, 
        IntakeRoller intakeRoller, 
        RollerFloor rollerFloor, 
        Shooter shooter) {

        this.joystick = joystick;

        this.autoaim = autoaim;

        this.subsystemManager = subsystemManager;

        this.drivetrain = drivetrain;
        this.hood = hood;
        this.indexer = indexer;
        this.intakeRack = intakeRack;
        this.intakeRoller = intakeRoller;
        this.rollerFloor = rollerFloor;
        this.shooter = shooter;
        
        configureBindings();
    }

    private void configureBindings() {    

        joystick.rightTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager)
        );

        joystick.rightBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORINGMOVING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager)
        );

        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.leftBumper().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_REVERSEINTAKING), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.povUp().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_TOWER_PRESET), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.povRight().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_TRENCH_PRESET), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.a().onTrue(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING_SYSTEMS_CHECK), subsystemManager)
        ).onFalse(
            Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED), subsystemManager)
        );

        joystick.x().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.IDLE), subsystemManager));

        joystick.povLeft().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_STOWED), subsystemManager));

        joystick.povDown().onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_IDLE), subsystemManager));
       
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        cojoystick.start().onTrue(new InstantCommand(HubShiftUtil::initialize));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
}
