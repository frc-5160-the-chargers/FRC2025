package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.KitBotCoralOut;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveDrive;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

@Logged
public class Kitbot extends TimedRobot implements LogLocal {

    private final SwerveDrive drivetrain = new SwerveDrive(SwerveConfigurator.DEFAULT_DRIVE_CONFIG);
    private final KitBotCoralOut kitbotCoralOut = new KitBotCoralOut(drivetrain);
    private final AutoFactory autoFactory = drivetrain.createAutoFactory();
    private final CommandXboxController controller = new CommandXboxController(4);

    public Kitbot() {
        Epilogue.bind(this);
        Monologue.setup(this, Epilogue.getConfig());
        log("TestPose3d", new Pose3d(0.0, 0, 0.5, Rotation3d.kZero));

        drivetrain.setDefaultCommand(drivetrain.stopCmd());

        if (RobotBase.isSimulation()) {
            SimulatedArena.getInstance().placeGamePiecesOnField();
            drivetrain.resetPose(new Pose2d(5, 7, Rotation2d.kZero));
        }
        log("hasInitialized", true);
        RobotModeTriggers.autonomous().onTrue(onePieceKitBot());
        controller.a().whileTrue(kitbotCoralOut.simulateHasCoral(drivetrain));
        controller.b().whileTrue(kitbotCoralOut.outtakeCmd());
    }

    public void robotPeriodic() {
        var startTime = System.nanoTime();
        //This actually checks the "calendar" occasionally to make sure there's something to do
        CommandScheduler.getInstance().run();
        SimulatedArena.getInstance().simulationPeriodic();
        log("simulatedCoralPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        log("simulatedAlgaePositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        log("loopRuntime", (System.nanoTime() - startTime) / 1e6);
    }

    public Command onePieceKitBot() {
        var routine = autoFactory.newRoutine("OnePieceKitBot");
        var kitBotSimTraj = routine.trajectory("KitBotSim");

        routine.active().onTrue(
            kitBotSimTraj.resetOdometry().andThen(
                kitBotSimTraj.cmd(),

                // The drivetrain is "booked" for this whole sequence
                // So we need to lock it into place so it doesn't drift
                drivetrain.stopCmd(),
                kitbotCoralOut.outtakeCmd()
            ).withName("AutoCommand")
        );

        return routine.cmd();
    }
}
