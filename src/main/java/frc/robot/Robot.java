package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.chargers.hardware.motorcontrol.Motor;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.UtilExtensionMethods;
import lombok.experimental.ExtensionMethod;
import monologue.LogLocal;
import monologue.Monologue;
import org.ironmaple.simulation.SimulatedArena;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;

@ExtensionMethod({UtilExtensionMethods.class})
public class Robot extends TimedRobot implements LogLocal {
    @Logged private final Motor motor = new SimMotor(DCMotor.getNEO(1), 1.0, KilogramSquareMeters.of(0.004));
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0198);
    
    @Logged
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }
    
    public Robot() {
        Epilogue.bind(this);
        Monologue.setup(this, Epilogue.getConfig());
        var routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                state -> log("SysIdRoutineState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setVoltage(voltage.in(Volts)),
                log -> {},
                new Subsystem() {}
            )
        );
        
        motor.setVelocityPID(new PIDConstants(0.5, 0,0));
        
        autonomous().onTrue(
//            routine.quasistatic(kForward).andThen(
//                routine.quasistatic(kReverse),
//                routine.dynamic(kForward),
//                routine.dynamic(kReverse)
//            )
           Commands.run(() -> motor.setVelocity(2.0, ff.calculate(2.0)))
        );
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (isSimulation()) SimulatedArena.getInstance().simulationPeriodic();
    }
}
