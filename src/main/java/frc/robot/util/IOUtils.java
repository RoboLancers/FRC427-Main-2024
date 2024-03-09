package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IOUtils {
    public static double get(String key, double defaultVal) {
        if (!SmartDashboard.containsKey(key)) SmartDashboard.putNumber(key, defaultVal); 
        return SmartDashboard.getNumber(key, defaultVal); 
    }

    public static double getNumber(String key) {
        return get(key, 0); 
    }

    public static boolean get(String key, boolean defaultVal) {
        if (!SmartDashboard.containsKey(key)) SmartDashboard.putBoolean(key, defaultVal); 
        return SmartDashboard.getBoolean(key, defaultVal); 
    }

    public static boolean getBoolean(String key) {
        return get(key, false); 
    }

    public static void set(String key, double value) {
        SmartDashboard.putNumber(key, value); 
    }
    
}
