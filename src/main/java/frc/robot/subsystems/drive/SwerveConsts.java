package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.chargers.misc.TunableValues.TunableNum;
import frc.robot.constants.ChoreoVars;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;

/**
 * Contains swerve drive constants that aren't already found in the TunerConstants file.
 */
public class SwerveConsts {
    public static final SwerveDrivetrainConstants DRIVE_CONSTS_CHOICE = TunerConstants.DrivetrainConstants;
    public static final SwerveModuleConstants<?, ?, ?>[] MODULE_CONSTS_CHOICES = {
        TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight
    };
    public static final DCMotor DRIVE_MOTOR_TYPE = DCMotor.getKrakenX60(1);
    public static final DCMotor STEER_MOTOR_TYPE = DCMotor.getKrakenX44(1);
    static final Distance BUMPER_WIDTH = Inches.of(3.5);
    static final double KT_AMPS_PER_NM = 1 / (
        DRIVE_MOTOR_TYPE.KtNMPerAmp / TunerConstants.FrontLeft.DriveMotorGearRatio
    );
    static final SwerveModulePosition[] EMPTY_POSITIONS = {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
    };

    static final TunableNum
        DEMO_POSE_X = new TunableNum("Swerve/demoPose/x", 0),
        DEMO_POSE_Y = new TunableNum("Swerve/demoPose/y", 0),
        DEMO_POSE_HEADING_DEG = new TunableNum("Swerve/demoPose/headingDeg", 0);

    static final TunableNum
        TRANSLATION_KP = new TunableNum("PathFollowing/TranslationKP", 8),
        ROTATION_KP = new TunableNum("PathFollowing/RotationKP", 8),
        ROTATION_KD = new TunableNum("PathFollowing/RotationKD", 0.02);

    static final Translation2d[] MODULE_TRANSLATIONS = {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
    static final Distance DRIVE_BASE_RADIUS = Meters.of(MODULE_TRANSLATIONS[0].getNorm());
    static final DriveTrainSimulationConfig MAPLESIM_CONFIG = DriveTrainSimulationConfig.Default()
        .withRobotMass(ChoreoVars.botMass)
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
            ChoreoVars.cof
        ));
}
