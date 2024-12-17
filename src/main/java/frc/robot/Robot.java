package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.chargers.hardware.encoders.VoidEncoder;
import frc.chargers.hardware.motorcontrol.SimMotor;
import frc.chargers.utils.UtilExtensionMethods;
import frc.chargers.utils.UtilMethods;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveModule;
import lombok.experimental.ExtensionMethod;
import org.ironmaple.simulation.SimulatedArena;

import javax.swing.text.html.Option;

import java.util.Optional;

import static edu.wpi.first.epilogue.Logged.Strategy.OPT_IN;

@Logged(strategy = OPT_IN)
@ExtensionMethod({UtilExtensionMethods.class})
public class Robot extends TimedRobot {
    @Logged private final SwerveModule mod = new SwerveModule(
        SwerveConfigurator.getDefaultConfig(),
        new VoidEncoder(),
        new SimMotor(DCMotor.getNEO(1), 150.0 / 7.0, Units.KilogramSquareMeters.of(0.004))
            .also(it -> {
                it.setPositionPID(new PIDConstants(0.7, 0.0, 0.01));
            }),
        new SimMotor(DCMotor.getNEO(1), 150.0 / 7.0, Units.KilogramSquareMeters.of(0.004)),
        Optional.empty()
    );
    
    @Logged
    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }
    
    public Robot() {
        Epilogue.bind(this);
        UtilMethods.configureDefaultLogging();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (isSimulation()) SimulatedArena.getInstance().simulationPeriodic();
        mod.setDesiredState(
            new SwerveModuleState(10, Rotation2d.k180deg),
            false
        );
    }
}
