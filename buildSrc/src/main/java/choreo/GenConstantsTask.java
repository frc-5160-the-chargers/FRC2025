package choreo;

import org.gradle.api.DefaultTask;
import org.gradle.api.provider.Property;
import org.gradle.api.tasks.Input;
import org.gradle.api.tasks.Optional;
import org.gradle.api.tasks.TaskAction;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.*;

import static choreo.Internals.*;

public abstract class GenConstantsTask extends DefaultTask {
    @Input @Optional public abstract Property<String> getChorFileDir();
    @Input @Optional public abstract Property<String> getOutputFilePackage();
    @Input @Optional public abstract Property<Boolean> getGenVariablesFile();
    @Input @Optional public abstract Property<String> getJavaRoot();
    @Input public abstract Property<String> getChorFileName();

    private final JSONParser jsonParser = new JSONParser();

    @TaskAction
    public void run() {
        var chorFilePath = getChorFileDir().getOrElse("src/main/deploy/choreo") + "/" + getChorFileName().get();
        var projectJson = getJsonContent(jsonParser, chorFilePath);

        genDocumentConstsFile(
            (JSONObject) projectJson.get("config"),
            projectJson.get("type").equals("Swerve")
        );
        if (getGenVariablesFile().getOrElse(false)) {
            var variablesJson = (JSONObject) projectJson.get("variables");
            genChoreoVarsFile(
                (JSONObject) variablesJson.get("expressions"),
                (JSONObject) variablesJson.get("poses")
            );
        }
    }

    private void genDocumentConstsFile(JSONObject config, boolean isSwerve) {
        var classPackage = getOutputFilePackage().getOrElse("choreo");
        var outputFilePath = getJavaRoot().getOrElse("src/main/java")
                + "/"
                + classPackage.replace(".", "/")
                + "/ChoreoConsts.java";
        var outputFile = new File(outputFilePath);
        outputFile.getParentFile().mkdirs();
        try (var out = new PrintWriter(outputFile)) {
            out.println("package " + classPackage + ";");
            out.println("""
            
            import edu.wpi.first.math.geometry.Translation2d;
            import edu.wpi.first.units.measure.*;
            import static edu.wpi.first.units.Units.*;
            
            /**
             * File containing values filled into the document settings for your choreo project.
             * This allows for modifying constants in choreo while keeping your robot code up-to-date.
             * DO NOT MODIFY this file yourself, as it is auto-generated.
             */
            public final class ChoreoConsts {""");

            double gearing = readNum(config, "gearing");
            double mass = readNum(config, "mass");
            double cof = readNum(config, "cof");
            double wheelRadius = readNum(config, "radius");
            double vmax = readNum(config, "vmax");
            double tmax = readNum(config, "tmax");
            double moi = readNum(config, "inertia");
            double maxLinearVel = vmax / gearing * wheelRadius;
            double maxWheelForce = tmax * gearing * 4 / wheelRadius;
            double maxLinearAccel = maxWheelForce / mass;

            write(out, "gearing", gearing);
            write(out, "frictionCoefficient", cof);
            write(out, "wheelRadius", wheelRadius, "Distance", "Meters");
            write(out, "moi", moi, "MomentOfInertia", "KilogramSquareMeters");
            write(out, "mass", mass, "Mass", "Kilograms");
            write(out, "driveMotorMaxTorque", tmax, "Torque", "NewtonMeters");

            var bumperConfig = (JSONObject) config.get("bumper");
            double front = readNum(bumperConfig, "front");
            double back = readNum(bumperConfig, "back");
            double side = readNum(bumperConfig, "side");
            write(out, "wheelBaseWithBumpers", front + back, "Distance", "Meters");
            write(out, "trackWidthWithBumpers", side * 2, "Distance", "Meters");

            if (isSwerve) {
                double frontLeftX = readNum((JSONObject) config.get("frontLeft"), "x");
                double frontLeftY = readNum((JSONObject) config.get("frontLeft"), "y");
                double backLeftX = readNum((JSONObject) config.get("backLeft"), "x");
                double backLeftY = readNum((JSONObject) config.get("backLeft"), "y");
                out.println(String.format("""
                    public static final Translation2d[] moduleTranslations = {
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                    };
                """, frontLeftX, frontLeftY, frontLeftX, -frontLeftY, backLeftX, backLeftY, backLeftX, -backLeftY));

                double drivebaseRadius = Math.hypot(frontLeftX, frontLeftY);
                double frictionFloorForce = mass * 9.81 * cof;
                double minLinearForce = Math.min(frictionFloorForce, maxWheelForce);
                double maxAngularVel = maxLinearVel / drivebaseRadius;
                double maxAngularAccel = minLinearForce * drivebaseRadius / moi; // I * alpha = F x R -> alpha = F x R / I
                write(out, "maxAngularVel", maxAngularVel, "AngularVelocity", "RadiansPerSecond");
                write(out, "maxAngularAccel", maxAngularAccel, "AngularAcceleration", "RadiansPerSecondPerSecond");
            } else {
                write(out, "trackWidth", readNum(config, "differentialTrackWidth"), "Distance", "Meters");
                out.println();
            }
            write(out, "maxLinearVel", maxLinearVel, "LinearVelocity", "MetersPerSecond");
            write(out, "maxLinearAccel", maxLinearAccel, "LinearAcceleration", "MetersPerSecondPerSecond");
            out.println();
            out.println("    private ChoreoConsts() {}");
            out.println("}");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    // TODO
    private void genChoreoVarsFile(JSONObject expressions, JSONObject poses) {
        var classPackage = getOutputFilePackage().getOrElse("choreo");
        var outputFilePath = getJavaRoot().getOrElse("src/main/java")
                + "/"
                + classPackage.replace(".", "/")
                + "/ChoreoVars.java";
        var outputFile = new File(outputFilePath);
        outputFile.getParentFile().mkdirs();
        try (var out = new PrintWriter(outputFile)) {
            out.println("package " + classPackage + ";");
            out.println("""
            
            import edu.wpi.first.math.geometry.Pose2d;
            import edu.wpi.first.math.geometry.Rotation2d;
            import edu.wpi.first.units.measure.*;
            import static edu.wpi.first.units.Units.*;
            
            /**
             * File containing variables created in choreo,
             * allowing for value changes in choreo to be reflected in robot code.
             * DO NOT MODIFY this file yourself, as it is auto-generated.
             */
            public final class ChoreoVars {""");
            for (var expName: expressions.keySet()) {
                var expJson = (JSONObject) expressions.get(expName);
                var unitData = UnitData.from(expJson.get("dimension").toString());
                if (unitData.isEmpty()) continue;
                if (unitData.get().dimension().equals("Number")) {
                    write(out, expName.toString(), readNum(expJson, "var"));
                } else {
                    write(
                        out, expName.toString(), readNum(expJson, "var"),
                        unitData.get().dimension(), unitData.get().baseUnit()
                    );
                }
            }
            out.println();
            out.println("    public static final class Poses {");
            for (var poseName: poses.keySet()) {
                var poseJson = (JSONObject) poses.get(poseName);
                double x = readNum(poseJson, "x");
                double y = readNum(poseJson, "y");
                double headingRad = readNum(poseJson, "heading");
                var headingExp = Math.abs(headingRad) < 1e-5
                        ? "Rotation2d.kZero"
                        : ("Rotation2d.fromRadians(" + headingRad + ")");
                out.println(
                    String.format(
                        "        public static final Pose2d %s = new Pose2d(%s, %s, %s);",
                        poseName, x, y, headingExp
                    )
                );
            }
            out.println();
            out.println("        private Poses() {}");
            out.println("    }");
            out.println();
            out.println("    private ChoreoVars() {}");
            out.println("}");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}