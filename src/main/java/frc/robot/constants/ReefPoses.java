package frc.robot.constants;


import edu.wpi.first.math.geometry.Pose2d;
import frc.chargers.data.CurrAlliance;

import java.util.List;

import static choreo.ChoreoVars.Poses.*;
import static choreo.util.ChoreoAllianceFlipUtil.flip;

/** Calculates the closest reef pose to the robot. */
public class ReefPoses {
    /** An enum used to describe which pole(left or right) to align on for a "face" on the hexagonal reef. */
    public enum ReefSide {
        LEFT, RIGHT
    }

    private static final List<Pose2d> ALL_POSES = List.of(
        reef0, reef1, reef2, reef3, reef4, reef5,
        reef6, reef7, reef8, reef9, reef10, reef11
    );

    public static Pose2d get(int pos) {
        var pose = ALL_POSES.get(pos);
        return CurrAlliance.red() ? flip(pose) : pose;
    }

    public static Pose2d getClosest(ReefSide side, Pose2d current) {
        if (CurrAlliance.red()) current = flip(current);
        var closest = reef0;
        double smallestTranslationDiff = 1000;
        for (int i = (side == ReefSide.LEFT ? 0 : 1); i < 12; i += 2) {
            var pose = ALL_POSES.get(i);
            double rotationDiff = Math.abs(current.getRotation().getDegrees() - pose.getRotation().getDegrees());
            if (rotationDiff > 45) {
                continue;
            }
            double translationDiff = current.getTranslation().getDistance(pose.getTranslation());
            if (translationDiff < smallestTranslationDiff) {
                smallestTranslationDiff = translationDiff;
                closest = pose;
            }
        }
        return CurrAlliance.red() ? flip(closest) : closest;
    }
}
