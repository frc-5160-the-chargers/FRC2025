package choreo;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.PrintWriter;
import java.util.Optional;

class Internals {
    static void write(PrintWriter out, String propertyName, double value) {
        out.println(
            String.format("    public static final double %s = %s;", propertyName, value)
        );
    }

    static void write(
        PrintWriter out,
        String propertyName,
        double siValue,
        String type,
        String siUnit
    ) {
        out.println(String.format("    public static final %s %s = %s.of(%s);", type, propertyName, siUnit, siValue));
    }

    static double readNum(JSONObject parent, String jsonKey) {
        return (Double) ((JSONObject) parent.get(jsonKey)).get("val");
    }

    static JSONObject getJsonContent(JSONParser jsonParser, String filePath) {
        try {
            BufferedReader br = new BufferedReader(new FileReader(filePath));
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }
            br.close();
            return (JSONObject) jsonParser.parse(fileContentBuilder.toString());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    record UnitData(String dimension, String baseUnit) {
         static Optional<UnitData> from(String choreoDimensionName) {
            return Optional.ofNullable(
                switch (choreoDimensionName) {
                    case "LinAcc" -> new UnitData("LinearAcceleration", "MetersPerSecondPerSecond");
                    case "LinVel" -> new UnitData("LinearVelocity", "MetersPerSecond");
                    case "Length" -> new UnitData("Distance", "Meters");
                    case "Angle" -> new UnitData("Angle", "Radians");
                    case "AngVel" -> new UnitData("AngularVelocity", "RadiansPerSecond");
                    case "AngAcc" -> new UnitData("AngularAcceleration", "RadiansPerSecond");
                    case "Time" -> new UnitData("Time", "Seconds");
                    case "Mass" -> new UnitData("Mass", "Kilograms");
                    case "Number" -> new UnitData("Number", "");
                    default -> null;
                }
            );
        }
    }
}
