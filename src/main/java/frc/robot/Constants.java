/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/** 
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Drivebase_Constants {

        public static final int rightMasterSpark_ID = 1;
        public static final int rightSlave1Spark_ID = 2;
        public static final int rightSlave2Spark_ID = 3;
        
        public static final int leftMasterSpark_ID = 9;
        public static final int leftSlave1Spark_ID = 10;
        public static final int leftSlave2Spark_ID = 11;

        public static final int SparkMax_CurrentLimit = 60;

        public static final double kTrackwidthMeters = .68;

        public static final double kMaxSpeedMetersPerSecond = 2.438;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double distPerPulse = .0011114506;
        public static final double velocityConversion = .0007780154;

        public static final int positionConversionFactor = 10;  //NEED TO SET FOR REALS

        public static class PID_Values {

            public static final double ksVolts = 0.361;
            public static final double kvVoltSecondsPerMeter = 3.73;
            public static final double kaVoltSecondsSquaredPerMeter = 0.55;

            public static final double kPDriveVel = 16.9;
 

        }
    }

    public static class Shooter_Constants
    {

        public static final int flywheelOneSpark_ID = 4;
        public static final int flywheelTwoSpark_ID = 8;
        public static final int kickerSpark_ID = 5;
        public static final int hoodTalon_ID = 12;

        public static final int maxRPM = 4900;

        public static class PID_Values  {

            public static final double kP = .4;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kIz = 0;
            public static final double kFF = 0.01;
            public static final int kMax = 1;
            public static final int kMin = -1;

        }

    }

    public static class Limelight_Constants
    {
        public static final double limelightAngle = 30; //NEED TO CALCULATE IN DEGREES
        public static final double targetHeight = 98; //NEED TO MEASURE IN INCHES
        public static final double limelightHeight = 22.5; //NEED TO MEASURE IN INCHES

        public static class PID_Values
        {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static class Arm_Constants
    {
        public static final int collectorTalonID = 11;
        public static final int leftArmSpark_ID = 8;
        public static final int rightArmSpark_ID = 9;
    }
}
