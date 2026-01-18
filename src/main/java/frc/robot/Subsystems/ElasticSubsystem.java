package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticSubsystem extends SubsystemBase {

    SendableChooser<String> autoChooser = new SendableChooser<>();

    public static void putNumber(String name, double number) {
        SmartDashboard.putNumber(name, number);
    }

    public static double getNumber(String name) {
        return SmartDashboard.getNumber(name, -1);
    }

    public static void putColor(String name, Color c) {
        SmartDashboard.putString(name, c.toHexString());
    }

    public static void putBoolean(String name, boolean b) {
        SmartDashboard.putBoolean(name, b);
    }

    public static boolean getBoolean(String name) {
        return SmartDashboard.getBoolean(name, false);
    }

    public static void putString(String name, String data) {
        SmartDashboard.putString(name, data);
    }

    public void putAutoChooser() {
        autoChooser.setDefaultOption("Nothing", "Nothing");
        autoChooser.addOption("Move left", "Move left");
        autoChooser.addOption("Move right", "Move right");
        autoChooser.addOption("Tag26_1.5m", "Tag26_1.5m");
        SmartDashboard.putData("Auton Selector", autoChooser);
    }

    public String getSelectedAuto() {
        return autoChooser.getSelected();
    }

}
