package frc.robot.Utilites;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

public class HelperFunctions {

    public static double clamp(double value, double min, double max) {
        return MathUtil.clamp(value, min, max);
    }

    public static double map(double input, double inMin, double inMax, double outMin, double outMax) {
        return (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static Color convertToGRB(Color rgbColor) {
        return new Color(rgbColor.green, rgbColor.red, rgbColor.blue);
    }

    public static Transform2d translationToTransform(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    public static Transform2d translationToTransform(double x, double y) {
        return new Transform2d(new Translation2d(x, y), new Rotation2d());
      }

}
