package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SubsystemManager.RobotState;

public class Autos {

    private final AutoFactory autoFactory;

    private final SubsystemManager subsystemManager;

    public Autos(AutoFactory autoFactory, SubsystemManager subsystemManager) {

        this.autoFactory = autoFactory;

        this.subsystemManager = subsystemManager;
    }

    public AutoRoutine LT_DoubleSweepReturn() {
        
        AutoRoutine routine = autoFactory.newRoutine("LT_DoubleSweepReturn");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1");
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2");
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3");

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RT_DoubleSweepReturn() {
        
        AutoRoutine routine = autoFactory.newRoutine("RT_DoubleSweepReturn");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1").mirrorY();
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2").mirrorY();
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3").mirrorY();

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    
    public AutoRoutine LT_DoubleSweepReturnSecondScoopCloseToHub() {
        
        AutoRoutine routine = autoFactory.newRoutine("LT_DoubleSweepReturnSecondScoopCloseToHub");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1");
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V2");
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3");

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RT_DoubleSweepReturnSecondScoopCloseToHub() {
        
        AutoRoutine routine = autoFactory.newRoutine("RT_DoubleSweepReturnSecondScoopCloseToHub");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1").mirrorY();
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V2").mirrorY();
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3").mirrorY();

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    
    public AutoRoutine LT_DoubleSweepReturnSecondScoopShorter() {
        
        AutoRoutine routine = autoFactory.newRoutine("LT_DoubleSweepReturnSecondScoopShorter");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1");
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V3");
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3");

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RT_DoubleSweepReturnSecondScoopShorter() {
        
        AutoRoutine routine = autoFactory.newRoutine("RT_DoubleSweepReturnSecondScoopShorter");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1").mirrorY();
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V3").mirrorY();
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3").mirrorY();

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine LT_DoubleSweepReturnSecondScoopLoop() {
        
        AutoRoutine routine = autoFactory.newRoutine("LT_DoubleSweepReturnSecondScoopLoop");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1");
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V4");
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3");

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine RT_DoubleSweepReturnSecondScoopLoop() {
        
        AutoRoutine routine = autoFactory.newRoutine("RT_DoubleSweepReturnSecondScoopLoop");
        AutoTrajectory path1 = routine.trajectory("LT_DoubleSweepReturn_1").mirrorY();
        AutoTrajectory path2 = routine.trajectory("LT_DoubleSweepReturn_2_V4").mirrorY();
        AutoTrajectory path3 = routine.trajectory("LT_DoubleSweepReturn_3").mirrorY();

        this.assignStandardCommandsToTrajectory(path1);
        this.assignStandardCommandsToTrajectory(path2);
        this.assignStandardCommandsToTrajectory(path3);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path2.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path3.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine M_Preload() {
        
        AutoRoutine routine = autoFactory.newRoutine("M_Preload");
        AutoTrajectory path1 = routine.trajectory("M_PreloadShot_1");

        this.assignStandardCommandsToTrajectory(path1);

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)),
                path1.cmd(),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_SCORING)),
                Commands.waitSeconds(3),
                Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED))                
            )
        );

        return routine;
    }

    private void assignStandardCommandsToTrajectory(AutoTrajectory path) {
        path.atTime("Intake").onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKING)));
        path.atTime("IntakeStop").onTrue(Commands.runOnce(() -> subsystemManager.setRobotState(RobotState.DRIVING_INTAKEDEPLOYED)));
    }
}
