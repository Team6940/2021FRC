// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.drive.commands.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.BallCmd;
import frc.robot.subsystems.intake.commands.Ballin;
import frc.robot.subsystems.intake.commands.Ballout;
import frc.robot.subsystems.intake.commands.Solenoidin;
import frc.robot.subsystems.intake.commands.Solenoidout;
import frc.robot.subsystems.limelight.photonlime;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.subsystems.balltrans.BallTrans;
import frc.robot.subsystems.balltrans.commands.BallTransCmd;
import frc.robot.subsystems.colorsensor.ColorSensor;
import frc.robot.subsystems.colorsensor.commands.getcolor;
import frc.robot.subsystems.colorsensor.commands.matchcolor;
import frc.robot.subsystems.colorsensor.commands.turnpanel;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //define Joystick
  public static XboxController m_joystick;

  //define fast&slow button
  public static JoystickButton fastButton;
  public static JoystickButton slowButton;

  // limelight button
  public static JoystickButton limelightButton;

  // shooter and balltrans button
  public static JoystickButton shooterButton;
  public static JoystickButton balltransButton;

  // intake button
  public static JoystickButton pushintakebutton;
  public static JoystickButton backintakebutton;
  public static JoystickButton ballstartbutton;

  // color sensor button
  public static JoystickButton matchcolorbutton;
  public static JoystickButton turnpanelbutton;

  // The robot's subsystems and commands are defined here...

  // DriveBase
  public static Drive m_Drive;
  public Command m_autoCommand;

  // Shooter
  public static Shooter m_Shooter;

  //ball transport
  public static BallTrans m_BallTrans;

  // intake
  public static Intake m_intake;

  //color sensor
  public static ColorSensor m_colorsensor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // set Joystick
  m_joystick = new XboxController(0);
 
  //set Joystick buttons
  //fastButton = new JoystickButton(m_driverjoystick, 3);
  //slowButton = new JoystickButton(m_driverjoystick, 4);
  limelightButton = new JoystickButton(m_joystick, 4);
  pushintakebutton = new JoystickButton(m_joystick, 1);
  backintakebutton = new JoystickButton(m_joystick, 3);
  ballstartbutton = new JoystickButton(m_joystick, 2);
  matchcolorbutton = new JoystickButton(m_joystick, 7);
  turnpanelbutton = new JoystickButton(m_joystick, 8);

  //set subsystems
  m_Drive = new Drive(); 
  m_Shooter = new Shooter();
  m_BallTrans = new BallTrans();
  m_intake = new Intake();
  m_colorsensor = new ColorSensor();
  
  //Set default command
  m_Drive.setDefaultCommand(new DriveCmd());
  m_Shooter.setDefaultCommand(new ShootCmd());
  m_BallTrans.setDefaultCommand(new BallTransCmd());
  m_colorsensor.setDefaultCommand(new getcolor());
  m_intake.setDefaultCommand(new BallCmd());
  // Configure the button bindings

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //fastButton.whenPressed(new setThreshFast());
    //slowButton.whenPressed(new setThreshSlow());
    limelightButton.whenHeld(new photonlime());
    matchcolorbutton.whenHeld(new matchcolor());
    turnpanelbutton.whenPressed(new turnpanel());
    pushintakebutton.whenPressed(new Solenoidout());
    backintakebutton.whenPressed(new Solenoidin());
    ballstartbutton.whenHeld(new Ballin());
    ballstartbutton.whenReleased(new Ballout());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = Robot.trajectory;
        /*TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);*/

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_Drive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            m_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_Drive::tankDriveVolts,
            m_Drive);

    // Reset odometry to the starting pose of the trajectory.
    m_Drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_Drive.tankDriveVolts(0, 0));
  }
}
