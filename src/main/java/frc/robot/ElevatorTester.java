package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.chargers.utils.InputStream;
import frc.chargers.utils.StatusSignalRefresher;
import frc.chargers.utils.TunableValues;
import frc.robot.subsystems.Elevator;
import monologue.LogLocal;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

import static monologue.Monologue.GlobalLog;

@Logged
public class ElevatorTester extends TimedRobot implements LogLocal {
	private final Elevator elevator = new Elevator();
	private final CommandXboxController controller = new CommandXboxController(0);
	private final InputStream controllerInput =
		InputStream.of(controller::getLeftY)
			.deadband(0.1, 1)
			.times(0.4);
	
	public ElevatorTester() {
		// Required for ChargerTalonFX and ChargerCANcoder to work
		StatusSignalRefresher.startPeriodic(this);
		// logging setup(required)
		Epilogue.bind(this);
		Monologue.setup(this, Epilogue.getConfig());
		GlobalLog.enableCommandLogging();
		DataLogManager.start();
		URCL.start();
		TunableValues.setTuningMode(true);
		addPeriodic(CommandScheduler.getInstance()::run, 0.02);
		
		controller.x()
			.whileTrue(elevator.moveToDemoHeightCmd());
		elevator.setDefaultCommand(elevator.setPowerCmd(controllerInput));
	}
	
	@Override
	public void testInit() {
		elevator.sysIdCmd().schedule();
	}
}
