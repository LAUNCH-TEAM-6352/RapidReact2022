// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Adds extensions to the standard SmartDashboard class. */
public final class SmartDashboardEx
{
    private static boolean isThrottled = false;

    /**
     * Sets if output to the SmartDashboard should be throttled.
     * 
     * @param isThrottled If output to the SmartDashboard should be throttled.
     */
    public static void setIsThrottled(boolean isThrottled)
    {
        SmartDashboardEx.isThrottled = isThrottled;
    }

    /**
     * Indicates if SmartDashboard output is throttled.
     * 
     * @return {@code true} if SmartDashboard output is throttled.
     */
    public static boolean isThrottled()
    {
        return isThrottled;
    }

    public static boolean getBoolean(String key, boolean defaultValue)
    {
        return SmartDashboard.getBoolean(key, defaultValue);
    }

    public static double getNumber(String key, double defaultValue)
    {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public static void putBoolean(String key, boolean value, boolean always)
    {
        if (always || !isThrottled)
        {
            SmartDashboard.putBoolean(key, value);
        }
    }

    public static void putBoolean(String key, boolean value)
    {
        putBoolean(key, value, false);
    }

    public static void putData(Sendable value, boolean always)
    {
        if (always || !isThrottled)
        {
            SmartDashboard.putData(value);
        }
   }

    public static void putData(Sendable value)
    {
        putData(value, false);
    }

    public static void putData(String key, Sendable value, boolean always)
    {
        if (always || !isThrottled)
        {
            SmartDashboard.putData(key, value);
        }
   }

    public static void putData(String key, Sendable value)
    {
        putData(key, value, false);
    }

    public static void putNumber(String key, double value, boolean always)
    {
        if (always || !isThrottled)
        {
            SmartDashboard.putNumber(key, value);
        }
    }

    public static void putNumber(String key, double value)
    {
        putNumber(key, value, false);
    }
}
