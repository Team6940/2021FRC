/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants {
    private static final double PI = Math.PI;

    public class Drivebase{

        // motor
        public static final int DRIVE_L1_PORT = 1;
        public static final int DRIVE_L2_PORT = 2;
        public static final int DRIVE_R1_PORT = 3;
        public static final int DRIVE_R2_PORT = 4;
        
        // initial speed
        public static final double Initial_Speed = 0.5;

        // "soft boot" parameter
        public static final double Loop_Parameter = 10;

        // unit conversion
        public static final double kDriveTick2Feet = 1.0 / 2048  * 6 * Math.PI / 12;


    }

    public class Limelight{
        // ledmode
        public static final int LED_DEFAULT = 0;
        public static final int LED_OFF = 1;
        public static final int LED_FLASH = 2;
        public static final int LED_ON = 3;

        // Limelight parameters
        /*These numbers must be tuned for your Robot! Be careful!*/
        public static final double STEER_K = 0.03;             // how hard to turn toward the target
        public static final double DRIVE_K = 0.8;              // how hard to drive fwd toward the target
        public static final double DESIRED_TARGET_AREA = 0.3;  // Area of the target when the robot reaches the wall
        public static final double MAX_DRIVE = 0.5;            // Simple speed limit so we don't drive too fast

        // The height bewtween the centre of target and the ground.The unit is meter
        public static final double Target_Height = 2.49555;

        // Set the angle bewteen Limelight and the ground.
        public static final double Limelight_Angle = PI/6;

        // The Height between Shooter and the ground .
        public static final double Shooter_Height = 0.45;

        public static final double StopLime_ThresholdLeft = -2.5;
        public static final double StopLime_ThresholdRght = 2.5;

        // Set PID parameters.
        public static final double KpAim = -0.1;
        public static final double KpDistance = -0.1;

        public static final double min_command = 0.05;
        

    }

    public class navX{

        // Balance Threshold
        public static final double kOffBalanceAngleThresholdDegrees = 10;
        public static final double kOonBalanceAngleThresholdDegrees  = 5;
    }

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;
    
        public static final int[] kLeftEncoderPorts = new int[] {0, 1};
        public static final int[] kRightEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
        }
      
    public static final class ShooterConstants {
        public static final int[] kEncoderPorts = new int[] {4, 5};
        public static final boolean kEncoderReversed = false;
        public static final int kEncoderCPR = 1024;
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations
            1.0 / (double) kEncoderCPR;
      
        public static final int kShooterLeftMotorPort = 5;
        public static final int kShooterRghtMotorPort = 6;

        public static final int kFeederMotorPort = 7;
      
        public static final double kShooterFreeRPS = 5300;
        public static final double kShooterTargetRPS = 4000;
        public static final double kShooterToleranceRPS = 50;
      
        // These are not real PID gains, and will have to be tuned for your specific robot.
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
      
        // On a real robot the feedforward constants should be empirically determined; these are
        // reasonable guesses.
        public static final double kSVolts = 0.05;
        public static final double kVVoltSecondsPerRotation =
            // Should have value 12V at free speed...
            12.0 / kShooterFreeRPS;
      
        public static final double kFeederSpeed = 0.5;
        }

        public static final class AutoConstants {
            public static final double kMaxSpeedMetersPerSecond = 3;
            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        
            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;
          }
    
    public class shooter{

        // motor
        public static final double ShooterON_Percent_Output = 0.6;
        public static final double ShooterOff_Percent_Output = 0.00;

        public static final boolean IS_LeftShooter_INVERTED = false;
        public static final boolean IS_RightShooter_INVERTED = true;
     
        public static final int Left_Shooter_Port = 5;
        public static final int Right_Shooter_Port = 6;

        // ball transpoer motor
        public static final int Left_Balltrans_Port = 7;
        public static final int Right_Balltrans_Port = 8;

        public static final boolean IS_Balltransleft_INVERTED = false;
        public static final boolean IS_BalltransRight_INVERTED = true;

        public static final double BallTrans_On_Power = 1;
        public static final double BallTrans_Off_Power = 0;


    }

    public class Intake{

        // motor
        public static final int Left_Intake_Port = 9;
        public static final int Right_Intake_Port = 10;

        public static final boolean Is_Intakeleft_Inverted = true;
        public static final boolean Is_Intakeright_Inverted = false;

        // solenoid
        public static final int Left_Solenoid_Port = 1;
        public static final int Right_Solenoid_Port = 2;

        public static final boolean Solenoid_Stop = false;
        public static final boolean Soenoid_Start = true;

        // motor speed
        public static final double Intake_Start_Speed = 1;
        public static final double Intake_Stop_Speed = 0;
    }

    public class colorsensor{
        public static final int Tuner_Port = 11;
        public static final double turner_power = 0.5;
    }

}
