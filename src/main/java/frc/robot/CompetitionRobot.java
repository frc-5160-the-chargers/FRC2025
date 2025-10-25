package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.chargers.hardware.SignalBatchRefresher;
import frc.chargers.data.RobotMode;
import frc.chargers.misc.Tracer;
import frc.chargers.misc.TunableValues.TunableBool;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.constants.ReefPoses;
import frc.robot.constants.ReefPoses.ReefSide;
import frc.robot.components.controllers.DriverController;
import frc.robot.components.controllers.OperatorController;
import frc.robot.components.vision.AprilTagVision;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Setpoint;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static frc.chargers.commands.TriggerUtil.doubleClicked;

public class CompetitionRobot extends LoggedRobot {
    private static final boolean IS_REPLAY = false;
    private static final TunableBool FIELD_RELATIVE = new TunableBool("FieldRelative", true);

    private final DriverController driver = new DriverController();
    private final OperatorController operator = new OperatorController();

    private final SwerveDrive drivetrain = new SwerveDrive();
    private final AprilTagVision vision = new AprilTagVision(drivetrain::bestPose);

    private final GlobalState globalState = new GlobalState();
    private final Elevator elevator = new Elevator(globalState);
    private final Wrist wrist = new Wrist(globalState);
    private final Intake intake = new Intake(globalState);

    private final RobotCommands botCommands = new RobotCommands(drivetrain, intake, wrist, elevator);
    private final AutoCommands autoCommands = new AutoCommands(botCommands, drivetrain.createAutoFactory(), intake, drivetrain);

    private final AutoChooser autoChooser = new AutoChooser();
    private final AutoChooser testModeChooser = new AutoChooser();

    private final RobotVisualization viz = new RobotVisualization(drivetrain, intake, wrist, elevator);

    public CompetitionRobot() {
        setUseTiming(!IS_REPLAY);
        DriverStation.silenceJoystickConnectionWarning(true);
        if (IS_REPLAY) {
            Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter());
        LoggedPowerDistribution.getInstance();
        Logger.start();
        Tracer.enableSingleThreadedMode();

        logMetadata();
        mapTriggers();
        mapDefaultCommands();
        mapAutoModes();
        mapTestCommands();

        if (RobotMode.get() == RobotMode.SIM) drivetrain.resetPose(new Pose2d(5, 7, Rotation2d.kZero));

        CommandScheduler.getInstance().onCommandInitialize(c -> Logger.recordOutput("Commands/" + c.getName(), true));
        CommandScheduler.getInstance().onCommandFinish(c -> Logger.recordOutput("Commands/" + c.getName(), false));

        SmartDashboard.putData("AutoChooser", autoChooser);
        SmartDashboard.putData("TestChooser", testModeChooser);
    }

    @Override
    public void loopFunc() { // WARNING: Do not modify this method.
        Tracer.trace("main loop", super::loopFunc);
    }

    @Override
    public void robotPeriodic() {
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        SignalBatchRefresher.refreshAll();
        Tracer.trace("Odometry Update", this::updateOdometry);
        if (RobotMode.get() == RobotMode.SIM) {
            Tracer.trace("maple sim", SimulatedArena.getInstance()::simulationPeriodic);
        }
        viz.periodic();
    }

    private void updateOdometry() {
        var yaw = drivetrain.getGyroInputs().odoYawValues[0];
        var yawTimestamp = drivetrain.getGyroInputs().odoYawTimestamps[0];
        for (var estimate: vision.update(yaw, yawTimestamp)) {
            drivetrain.addVisionData(estimate);
        }
    }

    private void mapDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.driveCmd(driver.forwardOutput, driver.strafeOutput, driver.rotationOutput, true)
        );
        elevator.setDefaultCommand(elevator.setPowerCmd(operator.manualElevatorInput));
        intake.setDefaultCommand(intake.idleCmd());
        wrist.setDefaultCommand(wrist.setPowerCmd(operator.manualPivotInput));
    }

    private void mapTriggers() {
        new Trigger(() -> !FIELD_RELATIVE.get())
            .whileTrue(
                drivetrain.driveCmd(driver.forwardOutput, driver.strafeOutput, driver.rotationOutput, false)
            );
        driver.L1()
            .whileTrue(
                drivetrain.pathfindCmd(() -> ReefPoses.getClosest(ReefSide.LEFT, drivetrain.poseEstimate()))
            );
        driver.R1()
            .whileTrue(
                drivetrain.pathfindCmd(() -> ReefPoses.getClosest(ReefSide.RIGHT, drivetrain.poseEstimate()))
            );
        driver.povUp()
            .whileTrue(drivetrain.driveCmd(() -> 0.03, () -> 0, () -> 0, false).withName("nudge"));
        driver.povDown()
            .whileTrue(drivetrain.driveCmd(() -> -0.03, () -> 0, () -> 0, false).withName("nudge"));
        driver.povLeft()
            .whileTrue(drivetrain.driveCmd(() -> 0, () -> 0.03, () -> 0, false).withName("nudge"));
        driver.povRight()
            .whileTrue(drivetrain.driveCmd(() -> 0, () -> -0.03, () -> 0, false).withName("nudge"));
        doubleClicked(driver.touchpad())
            .onTrue(
                Commands.runOnce(() -> {
                    var currPose = drivetrain.bestPose();
                    drivetrain.resetPose(new Pose2d(currPose.getX(), currPose.getY(), Rotation2d.kZero));
                })
                .ignoringDisable(true)
                .withName("zero heading")
            );
        operator.povUp()
            .whileTrue(intake.outtakeForeverCmd());
        operator.povDown()
            .whileTrue(intake.intakeForeverCmd());
        operator.rightBumper()
            .whileTrue(botCommands.sourceIntake());
        operator.leftBumper()
            .whileTrue(botCommands.stow());
        operator.a()
            .whileTrue(botCommands.moveTo(Setpoint.score(1)));
        operator.b()
            .whileTrue(botCommands.moveTo(Setpoint.score(2)));
        operator.y()
            .whileTrue(botCommands.moveTo(Setpoint.score(3)));
        operator.x()
            .whileTrue(botCommands.moveTo(Setpoint.score(4)));
        doubleClicked(operator.start())
            .onTrue(Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getDemoPose())).ignoringDisable(true));
    }

    private void logMetadata() {
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", Integer.toString(BuildConstants.DIRTY));
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    }

    private void mapAutoModes() {
        autoChooser.addCmd("Taxi", autoCommands::taxi);
        autoChooser.addCmd("3x L4 Right", autoCommands::tripleL4South);
        autoChooser.addCmd("4x L1 Right", autoCommands::quadL1South);
        autoChooser.addCmd("L4 L1 L1 Right", autoCommands::l4L1L1South);
        autoChooser.addCmd("L4 L4 L1 Right", autoCommands::l4L4L1South);
        autoChooser.addCmd("One Piece L4", () -> autoCommands.onePieceL4(false));
        autoChooser.addCmd("One Piece L4(mirrored)", () -> autoCommands.onePieceL4(true));
        autoChooser.addCmd("One Piece L4(center)", autoCommands::onePieceL4Center);
        autoChooser.addCmd("One Piece L1", autoCommands::onePieceL1);
        autoChooser.addCmd("(TEST ONLY) figure eight", autoCommands::figureEight);
        autoChooser.addCmd("(TEST ONLY) multi piece", autoCommands::multiPieceTest);
        autoChooser.addCmd("Stupid fing reset pose test", autoCommands::resetOdoTest);

        SmartDashboard.putData("AutoChooser", autoChooser);
        autonomous().onTrue(autoChooser.selectedCommandScheduler());
    }

    private void mapTestCommands() {
        testModeChooser.addCmd("Simple Path", autoCommands::simplePath);
        testModeChooser.addCmd("Simple Path w/ rotate", autoCommands::simplePathWithRotate);

        testModeChooser.addCmd("MoveL4", () -> botCommands.moveTo(Setpoint.score(4)));
        testModeChooser.addCmd("MoveToDemoSetpoint", botCommands::moveToDemoSetpoint);
        testModeChooser.addCmd("MoveToCoralSetpoint", () -> wrist.setPowerCmd(() -> 1));
        testModeChooser.addCmd(
            "Outtake",
            () -> intake.setHasCoralInSimCmd(true).andThen(intake.outtakeCmd())
        );
        testModeChooser.addCmd("Score L4", () -> botCommands.scoreSequence(4));
        if (RobotMode.get() == RobotMode.SIM) {
            testModeChooser.addCmd(
                "Stow and simulate coral",
                () -> intake.setHasCoralInSimCmd(true).andThen(botCommands.stow())
            );
        }
        testModeChooser.addCmd("Move to L3", () -> botCommands.moveTo(Setpoint.score(3)));
        testModeChooser.addCmd(
            "Wheel radius characterization",
            drivetrain::wheelRadiusCharacterization
        );
        testModeChooser.addCmd(
            "Align(repulsor)",
            () -> drivetrain.pathfindCmd(() -> ReefPoses.get(8))
        );
        testModeChooser.addCmd(
            "Reset Pose",
            () -> Commands.runOnce(() -> drivetrain.resetPose(drivetrain.getDemoPose()))
        );
        testModeChooser.addCmd(
            "Set Steer angles",
            () -> drivetrain.setSteerAngles(Rotation2d.k180deg.plus(Rotation2d.kCW_90deg))
        );
        testModeChooser.addCmd(
            "Drive Steer Motors",
            drivetrain::runTurnMotors
        );
        testModeChooser.addCmd(
            "HelloThere",
            () -> drivetrain.pathfindCmd(() -> ReefPoses.getClosest(ReefSide.LEFT, drivetrain.poseEstimate()))
        );
        testModeChooser.addCmd(
            "DriveVoltages",
            drivetrain::runDriveMotors
        );

        SmartDashboard.putData("TestChooser", testModeChooser);
        test().onTrue(testModeChooser.selectedCommandScheduler().ignoringDisable(true));
    }
}
