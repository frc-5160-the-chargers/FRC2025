package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.chargers.misc.TunableValues.TunableNum;
import frc.robot.constants.TunerConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.simulation.drivesims.COTS.WHEELS.DEFAULT_NEOPRENE_TREAD;

/**
 * Most constants related to swerve are found in the TunerConstants file.
 */
public class SwerveConsts {
    public static final double ODO_FREQUENCY_HZ = 250.0;
    public static final DCMotor DRIVE_MOTOR_TYPE = DCMotor.getKrakenX60(1);
    public static final DCMotor STEER_MOTOR_TYPE = new DCMotor(
        12.0, 4.05, 275, 1.4,
        RPM.of(7530).in(RadiansPerSecond), 1
    ); // kraken x44

    static final SwerveSetpoint NULL_SETPOINT = new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[]{
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        },
        DriveFeedforwards.zeros(4)
    );

    static final TunableNum
        DEMO_POSE_X = new TunableNum("swerve/demoPose/x", 0),
        DEMO_POSE_Y = new TunableNum("swerve/demoPose/y", 0),
        DEMO_POSE_HEADING_DEG = new TunableNum("swerve/demoPose/headingDeg", 0),
        DEMO_DRIVE_VOLTS = new TunableNum("swerve/demoDriveVolts", 3),
        DEMO_STEER_VOLTS = new TunableNum("swerve/demoSteerVolts", 3);

    static final Distance BUMPER_WIDTH = Inches.of(3.5);
    static final Mass ROBOT_MASS = Pounds.of(116);
    static final MomentOfInertia ROBOT_BODY_MOI = KilogramSquareMeters.of(6.283);
    static final double FORCE_KT = 0;

    static final Translation2d[] MODULE_TRANSLATIONS = {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
    static final DriveTrainSimulationConfig MAPLESIM_CONFIG = DriveTrainSimulationConfig.Default()
        .withRobotMass(ROBOT_MASS)
        .withCustomModuleTranslations(MODULE_TRANSLATIONS)
        .withGyro(COTS.ofPigeon2())
        .withBumperSize(
            Meters.of(2 * (TunerConstants.FrontLeft.LocationX + BUMPER_WIDTH.in(Meters))),
            Meters.of(2 * (TunerConstants.FrontLeft.LocationY + BUMPER_WIDTH.in(Meters)))
        )
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DRIVE_MOTOR_TYPE,
            STEER_MOTOR_TYPE,
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
            Meters.of(TunerConstants.FrontLeft.WheelRadius),
            KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
            DEFAULT_NEOPRENE_TREAD.cof
        ));
    static final RobotConfig PATH_PLANNER_CONFIG = new RobotConfig(
        ROBOT_MASS.in(Kilograms),
        ROBOT_BODY_MOI.in(KilogramSquareMeters),
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            DEFAULT_NEOPRENE_TREAD.cof,
            DRIVE_MOTOR_TYPE.withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1
        ),
        MODULE_TRANSLATIONS
    );

    static final Distance DRIVEBASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());

    static final double TRANSLATION_KP = 8;
    static final double TRANSLATION_TOLERANCE = 0.014;
    static final double ROTATION_KP = 8;
    static final double ROTATION_KD = 0.01;
    static final double ROTATION_TOLERANCE = 0.05;
}
