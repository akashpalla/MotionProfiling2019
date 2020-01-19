/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class Constants {
    public static final double kMaxVeloctyMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSq = 2;
    public static final double kDriveGearRatio  = 11.24;
    public static final double kWheelRadiusMeters = .0762;
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    
    public static final double kTrackWidthMeters = .6159;    
    public static final double kTrackScrubFactor = 1.0469745223;
    public static final double kEpsilon = 1E-9;

}
