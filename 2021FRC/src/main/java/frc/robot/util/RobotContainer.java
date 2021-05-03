// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drive.commands.*;
import frc.robot.subsystems.limelight.limelighton;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.ShootCmd;
import frc.robot.subsystems.shooter.commands.ShooterOff;
import frc.robot.subsystems.shooter.commands.ShooterOn;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.Autonomous;
import frc.robot.subsystems.drive.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //define Joystick
  public static Joystick m_driverjoystick;
  public static Joystick m_operatorjoystick;

  //define fast&slow button
  public static JoystickButton fastButton;
  public static JoystickButton slowButton;
  public static JoystickButton limelightButton;
  public static JoystickButton shooterButton;
  public static JoystickButton testLimeButton;

  // The robot's subsystems and commands are defined here...

  // DriveBase
  public static Drive m_Drive;
  public Autonomous m_autoCommand;

  // Shooter
  public static Shooter m_Shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // set Joystick
  m_driverjoystick = new Joystick(0);
  m_operatorjoystick = new Joystick(1);
 
  //set Joystick buttons
  fastButton = new JoystickButton(m_driverjoystick, 3);
  slowButton = new JoystickButton(m_driverjoystick, 4);
  shooterButton = new JoystickButton(m_operatorjoystick, 1);
  limelightButton = new JoystickButton(m_driverjoystick, 5);

  //set subsystems
  m_Drive = new Drive(); 

  m_Shooter = new Shooter();
  
  //Set default command
  m_Drive.setDefaultCommand(new DriveCmd());
  m_Shooter.setDefaultCommand(new ShootCmd());

  //set autoCommand
  m_autoCommand = new Autonomous();

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
    fastButton.whenPressed(new setThreshFast());
    slowButton.whenPressed(new setThreshSlow());
    shooterButton.whenHeld(new ShooterOn());
    shooterButton.whenReleased(new ShooterOff());
    limelightButton.whenHeld(new limelighton());
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
