package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.LaserCanUtil;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.subsystems.CoralIntakePivot;
import frc.robot.subsystems.Elevator;
import monologue.LogLocal;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

@SuppressWarnings("FieldCanBeLocal")
@Logged
public class RandomTesting extends TimedRobot implements LogLocal {
	private final Elevator elevator = new Elevator();
	private final CommandXboxController controller = new CommandXboxController(0);
	private final InputStream elevatorInput =
		InputStream.of(controller::getLeftY)
			.deadband(0.1, 1)
			.times(0.4);
	
	private final CoralIntakePivot pivot = new CoralIntakePivot();
	private final InputStream pivotInput =
		InputStream.of(controller::getRightY)
			.signedPow(2)
			.times(0.3);
	
	public RandomTesting() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		URCL.start();
		DataLogManager.start();
		Monologue.setup(this, Epilogue.getConfig());
//		Epilogue.getConfig().backend = EpilogueBackend.multi(
//			new FileBackend(DataLogManager.getLog()),
//			new NTEpilogueBackend(NetworkTableInstance.getDefault())
//		);
//		GlobalLog.enableCommandLogging();
		TunableValues.setTuningMode(true);
		LaserCanUtil.setup(true);
		addPeriodic(CommandScheduler.getInstance()::run, 0.02);
		
		controller.x().whileTrue(elevator.moveToDemoHeightCmd());
		elevator.setDefaultCommand(elevator.setPowerCmd(elevatorInput));
		pivot.setDefaultCommand(pivot.setPowerCmd(pivotInput));
		controller.y().whileTrue(pivot.setDemoAngleCmd());
		controller.leftBumper()
			.onTrue(pivot.resetAngleToStowCmd());
		// kS, kG * cos(theta), kV, kA
		// voltage -> velocity, accel
	}
	
	@Override
	public void testInit() {
		elevator.setDemoVoltageCmd().schedule();
	}
}
