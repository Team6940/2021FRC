/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

public class Constants {

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

        // solenoid
        public static final int DRIVE_STATE_PORT = 6;

        // invert

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
        public static final double DRIVE_K = 0.26;             // how hard to drive fwd toward the target
        public static final double DESIRED_TARGET_AREA = 13.0; //Area of the target when the robot reaches the wall
        public static final double MAX_DRIVE = 0.7;            // Simple speed limit so we don't drive too fast

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
      
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
             // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
        }
      
    public static final class ShooterConstants {
        public static final int[] kEncoderPorts = new int[] {4, 5};
        public static final boolean kEncoderReversed = false;
        public static final int kEncoderCPR = 1024;
        public static final double kEncoderDistancePerPulse =
            // Distance units will be rotations
            1.0 / (double) kEncoderCPR;
      
        public static final int kShooterMotorPort = 4;
        public static final int kFeederMotorPort = 5;
      
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

    public class shooter{

        // motor
        public static final double ShooterON_Percent_Output = 0.6;
        public static final double ShooterOff_Percent_Output = 0.00;

        public static final boolean IS_LeftShooter_INVERTED = false;
        public static final boolean IS_RightShooter_INVERTED = true;
     
        public static final int Left_Shooter_Port = 5;
        public static final int Right_Shooter_Port = 6;
    }

    public class Gyro{

         // PID for gyro
         public static final double DRIVE_KP = 0.01;
         public static final double DRIVE_KI = 0.00;
         public static final double DRIVE_KD = 0.00;
         public static final double DRIVE_KF = 0.00;

         public static final double ABSOLUTE_TOLERANCE = 2.0;
         public static final double GYRO_MAX_INPUT = 180;
         public static final double GYRO_MAX_OUTPUT = 0.75;
         public static final boolean IS_GYRO_CONTINUOUS = true;

    }

    public class Input{

        // driver sticks are aero-sticks using JoystickWrapper
        // operator and climb sticks are X-box sticks
        public static final int DRIVER_Left_PORT = 0;
        public static final int DRIVER_Right_PORT = 1;
        public static final int OPERATOR_PORT = 2;
        public static final int CLIMB_STICK_PORT = 3;

        // driver left buttons
        public static final int LIMELIGHT_BUTTON = 1;

        // driver right buttons
        public static final int DRIVE_STRAIGHT_BUTTON = 2;
        public static final int SHIFT_DRIVESTATE_BUTTON = 1;

        // operator buttons
        public static final int INTAKE_DOWN_BUTTON = 1;
        public static final int INTAKE_UP_BUTTON = 2;
        public static final int CARGO_OUTPUT_BUTTON = 3;
        public static final int PANEL_IN_BUTTON = 5;
        public static final int PANEL_OUT_BUTTON= 6;

        // climb stick buttons
        public static final int ELEVATOR_FOR_CLIMB_BUTTON = 1;
        public static final int SWITCH_STATUS_BUTTON = 2;
        public static final int CLIMB_UP_BUTTON = 3;
        public static final int MOVE_FORWARD_BUTTON = 4;
        public static final int CLIMB_WITH_JOYSTICK_BUTTON = 5;
        public static final int TRIPLE_WITH_JOYSTICK_BUTTON = 6;

        // limit switch
        public static final int SWITCH_ELEVATOR_DOWN = 0;
        public static final int SWITCH_ELEVATOR_UP = 2;
        public static final int SWITCH_CARGO = 9;

        // operator pov
        public static final int ELEVATOR_DOWN_POV = 180;
        public static final int ELEVATOR_LEVEL1_POV = 90;
        public static final int ELEVATOR_LEVEL2_POV = 270;
        public static final int ELEVATOR_LEVEL3_POV = 0;

    }

    public class Elevator{

        // motors
        public static final int ELEVATOR_1_PORT = 5;
        public static final int ELEVATOR_2_PORT = 6;
        public static final int ELEVATOR_3_PORT = 7;
        public static final int ELEVATOR_4_PORT = 8;

        public static final boolean IS_E1_INVERTED = true;
        public static final boolean IS_E2_INVERTED = false;
        public static final boolean IS_E3_INVERTED = true;
        public static final boolean IS_E4_INVERTED = true;

        // PID
        public static final double ELEVATOR_P = 0.01;
        public static final double ELEVATOR_I = -0.0001;
        public static final double ELEVATOR_D = 0.00;
        public static final double ELEVATOR_F = 0.00;
        public static final int ELEVATOR_TIMEOUT = 30; // in millsecond

        // position
        public static final double ELEVATOR_LEVEL1_HEIGHT = 46000;
        public static final double ELEVATOR_LEVEL2_HEIGHT = 230000;
        public static final double ELEVATOR_LEVEL3_HEIGHT = 405000;
        public static final double ELEVATOR_CLIMB_HEIGHT = 300000;
        public static final double ELEVATOR_DOWN_HEIGHT = 0.00;

        public static final double ELEVATOR_DOWN_PERCENT_OUTPUT =  -0.5;

        // solenoid
        public static final int ELEVATOR_STATE_PORT = 3;

    }

    public class Intake{

        // motor
        public static final int INTAKE_RAIL_PORT = 9;
        public static final int INTAKE_CLAW_PORT = 10;

        public static final boolean IS_RAIL_LINVERTED = false;
        public static final boolean IS_CLAW_INVERTED = false;

        public static final double INTAKE_RAIL_PERCENT_OUTPUT = 0.75;
        public static final double INTAKE_CLAW_PERCENT_OUTPUT = 0.75;

        // solenoid
        public static final int INTAKE_SOLENOID_PORT = 2;

        public static final boolean IS_INTAKERAIL_INVERTED = true;

    }

    public class Output{

        // motor
        public static final int OUTPUT_CARGO_PORT = 11;
        public static final int OUTPUT_PANEL_PORT = 12;

        public static final boolean IS_CARGO_INVERTED = false;
        public static final boolean IS_PANEL_INVERTED = false;

        public static final double CARGO_PERCENT_OUTPUT = 0.75;
        public static final double PANEL_IN_PERCENT_OUTPUT = 0.65;
        public static final double PANEL_OUT_PORTION = -0.85;

        // solenoid
        public static final int OUTPUT_SOLENOID_PORT = 5;

    }

    public class Climb{

        // motor
        public static final int TRIPLE_LEFT_PORT = 13;
        public static final int TRIPLE_RIGHT_PORT = 14;
        public static final int CLIMB_FORWARD_PORT = 15;

        // solenoid
        public static final int CLIME_SOLENOID_PORT = 7;
        public static final int CLIME_OUT_PORT = 4;

        public static final boolean IS_TRIPLELEFT_INVERTED = false;
        public static final boolean IS_TRIPLERIGHT_INVERTED = true;
        public static final boolean IS_FORWARD_INVERTED = false;

        public static final double TRIPLE_DOWN_POSITION = 0.00;
        public static final double TRIPLE_UP_POSITION = 0.00;

        public static final double TRIPLE_KP = 0.01;
        public static final double TRIPLE_KI = 0.00;
        public static final double TRIPLE_KD = 0.00;
        public static final double TRIPLE_KF = 0.00;
        public static final int CLIMB_TIMEOUT = 30;

        public static final double CLIMB_UP_PERCENTOUTPUT = 0.25;
        public static final double CLIME_FORWARD_PERCENTOUTPUT = 0.5;

    }

    public class Vision{

        public static final double LIMELIGHT_HEIGHT = 1000;
        public static final double LIMELIGHT_ANGLE = -19;

        public static final double PANEL_REFLECTIVETAPE_HEIGHT = 0;
        public static final double CARGO_REFLECTIVETAPE_HEIGHT = 0;

        // PID for vision
        public static final double VISION_KP = 0.01;
        public static final double VISION_KI = 0.00;
        public static final double VISION_KD = 0.00;
        public static final double VISION_KF = 0.00;

        public static final double STEER_K = 0.0275;    // how hard to turn toward the target
        public static final double DRIVE_K = 0.00;       // how hard to drive fwd toward the target
        public static final double TARGET_AREA = 13.0;  // Area of the target when the robot reaches the wall
        public static final double MAX_DRIVE = 0.45;    // Simple speed limit so we don't drive too fast

    }

}
