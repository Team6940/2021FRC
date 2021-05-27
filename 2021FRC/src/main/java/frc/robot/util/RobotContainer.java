// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.drive.commands.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.BallCmd;
import frc.robot.subsystems.intake.commands.IntakeInvert;
import frc.robot.subsystems.intake.commands.Solenoidin;
import frc.robot.subsystems.intake.commands.Solenoidout;
import frc.robot.subsystems.limelight.photonlime;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.InvertShooter;
import frc.robot.subsystems.shooter.commands.ShootCmd;
import frc.robot.subsystems.vision.vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.auto.autoCmdAll;
import frc.robot.subsystems.balltrans.BallTrans;
import frc.robot.subsystems.balltrans.commands.BallTransCmd;
import frc.robot.subsystems.balltrans.commands.InvertBall;
import frc.robot.subsystems.colorsensor.ColorSensor;
import frc.robot.subsystems.colorsensor.commands.BackSolenoid;
import frc.robot.subsystems.colorsensor.commands.PushSolenoid;
import frc.robot.subsystems.colorsensor.commands.StopMotor;
import frc.robot.subsystems.colorsensor.commands.getcolor;
import frc.robot.subsystems.colorsensor.commands.matchcolor;
import frc.robot.subsystems.colorsensor.commands.turnpanel;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //define Joystick
  public static XboxController m_driverjoystick;
  public static XboxController m_operatorjoystick;

  // optimize driving buttons
  public static POVButton setForwardLittle;
  public static POVButton setBackLittle;
  public static POVButton setRghtLittle;
  public static POVButton setLeftLittle;

  // limelight button
  public static JoystickButton limelightButton;

  // shooter and balltrans button
  public static JoystickButton balltransButton;
  public static JoystickButton balltransinvert;
  public static JoystickButton shooterinvert;

  // intake button
  public static JoystickButton pushintakebutton;
  public static JoystickButton backintakebutton;
  public static JoystickButton ballstartbutton;
  public static JoystickButton intakeinvertButton;

  // color sensor button
  public static JoystickButton matchcolorbutton;
  public static JoystickButton turnpanelbutton;
  public static JoystickButton pushcolorsolenoid;
  public static JoystickButton backcolorsolenoid;
  public static JoystickButton stopcolorsolenoid;

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

  // USB camera
  public static vision m_usbcameragroup;

  // Timer
  public  static Timer m_timer;

  // all path trajectories
  public static TrajectoryLoader m_TrajectoryLoader;
  public static HashMap<String, Trajectory> m_trajectories;
  public static Trajectory m_trajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // set Joystick
  m_driverjoystick = new XboxController(0);
  m_operatorjoystick = new XboxController(1);
 
  //set Joystick buttons
  //fastButton = new JoystickButton(m_driverjoystick, 3);
  //slowButton = new JoystickButton(m_driverjoystick, 4);

  // Driver 2's button
  shooterinvert = new JoystickButton(m_operatorjoystick, 6);
  balltransinvert = new JoystickButton(m_operatorjoystick, 5);
  matchcolorbutton = new JoystickButton(m_operatorjoystick, 7);
  turnpanelbutton = new JoystickButton(m_operatorjoystick, 8);
  pushintakebutton = new JoystickButton(m_operatorjoystick, 1);
  backintakebutton = new JoystickButton(m_operatorjoystick , 3);
  stopcolorsolenoid = new JoystickButton(m_operatorjoystick, 2);

  //Driver 1's Button
  limelightButton = new JoystickButton(m_driverjoystick, 4);
  pushcolorsolenoid = new JoystickButton(m_driverjoystick, 6);
  backcolorsolenoid = new JoystickButton(m_driverjoystick, 5);
  intakeinvertButton = new JoystickButton(m_driverjoystick, 2);
  setForwardLittle = new POVButton(m_driverjoystick, 0);
  setRghtLittle = new POVButton(m_driverjoystick, 90);
  setBackLittle = new POVButton(m_driverjoystick, 180);
  setLeftLittle = new POVButton(m_driverjoystick, 270);

  //create timer
  m_timer = new Timer();
  m_timer.start();
  double time_cur = m_timer.get();
  SmartDashboard.putNumber("time_cur", time_cur);
  m_TrajectoryLoader = new TrajectoryLoader();
  m_trajectories = m_TrajectoryLoader.loadTrajectories();
  m_trajectory = getDefaultTrajectory();

  //set subsystems
  m_Drive = new Drive(); 
  m_Shooter = new Shooter();
  m_BallTrans = new BallTrans();
  m_intake = new Intake();
  m_colorsensor = new ColorSensor();
  m_usbcameragroup = new vision();
  
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

    // Limelight Button
    limelightButton.whenHeld(new photonlime());

    //Color sensor Button
    matchcolorbutton.whenHeld(new matchcolor());
    turnpanelbutton.whenPressed(new turnpanel());
    pushcolorsolenoid.whenPressed(new PushSolenoid());
    backcolorsolenoid.whenPressed(new BackSolenoid());;
    stopcolorsolenoid.whenPressed(new StopMotor());

    // Intake and shooter button
    pushintakebutton.whenHeld(new Solenoidout());
    backintakebutton.whenHeld(new Solenoidin());
    intakeinvertButton.whenHeld(new IntakeInvert());
    //ballstartbutton.whenHeld(new Ballin());
    //ballstartbutton.whenReleased(new Ballout());
    balltransinvert.whenHeld(new InvertBall());
    shooterinvert.whenHeld(new InvertShooter());

    // Optimize driving button
    setForwardLittle.whenHeld(new setForwardLittle());
    setForwardLittle.whenReleased(new setLittleOff());
    setBackLittle.whenHeld(new setBackLittle());
    setBackLittle.whenReleased(new setLittleOff());
    setRghtLittle.whenHeld(new setRghtLittle());
    setRghtLittle.whenReleased(new setLittleOff());
    setLeftLittle.whenHeld(new setLeftLittle());
    setLeftLittle.whenReleased(new setLittleOff());
    //setOptimizeButton1.whenHeld(new setModeTrue());
    //setOptimizeButton2.whenReleased(new setModeFalse());
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new autoCmdAll();
 }

  public Trajectory getDefaultTrajectory(){
    Trajectory trajectory;
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
    trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
      return trajectory;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand_old() {
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
