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

            writeDouble(out, config, "gearing", "gearing");
            writeDouble(out, config, "cof", "frictionCoefficient");
            writeMeasure(out, config, "radius", "wheelRadius", "Distance", "Meters");
            writeMeasure(out, config, "inertia", "moi", "MomentOfInertia", "KilogramSquareMeters");
            writeMeasure(out, config, "vmax", "maxVel", "AngularVelocity", "RadiansPerSecond");
            writeMeasure(out, config, "mass", "mass", "Mass", "Kilograms");
            writeMeasure(out, config, "tmax", "maxTorque", "Torque", "NewtonMeters");
            var bumperConfig = (JSONObject) config.get("bumper");
            double front = readVal(bumperConfig, "front");
            double back = readVal(bumperConfig, "back");
            double side = readVal(bumperConfig, "side");
            out.println("    public static final Distance wheelBaseWithBumpers = Meters.of(" + (front + back) + ");");
            out.println("    public static final Distance trackWidthWithBumpers = Meters.of(" + (side * 2) + ");");
            if (isSwerve) {
                double frontLeftX = readVal((JSONObject) config.get("frontLeft"), "x");
                double frontLeftY = readVal((JSONObject) config.get("frontLeft"), "y");
                double backLeftX = readVal((JSONObject) config.get("backLeft"), "x");
                double backLeftY = readVal((JSONObject) config.get("backLeft"), "y");
                out.println(String.format("""
                    public static final Translation2d[] moduleTranslations = {
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                        new Translation2d(%s, %s),
                    };
                """, frontLeftX, frontLeftY, frontLeftX, -frontLeftY, backLeftX, backLeftY, backLeftX, -backLeftY));
            } else {
                writeMeasure(out, config, "differentialTrackWidth", "trackWidth", "Distance", "Meters");
                out.println();
            }
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
                    writeDouble(out, expJson, "var", expName.toString());
                } else {
                    writeMeasure(
                        out, expJson, "var", expName.toString(),
                        unitData.get().dimension(), unitData.get().baseUnit()
                    );
                }
            }
            out.println();
            out.println("    public static final class Poses {");
            for (var poseName: poses.keySet()) {
                var poseJson = (JSONObject) poses.get(poseName);
                double x = readVal(poseJson, "x");
                double y = readVal(poseJson, "y");
                double headingRad = readVal(poseJson, "heading");
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