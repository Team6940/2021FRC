// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drive.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
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
  public static Joystick m_stickL;

  //define fast&slow button
  public static JoystickButton fastButton;
  public static JoystickButton slowButton;
  public static JoystickButton limelightButton;

  // The robot's subsystems and commands are defined here...
  public static Drive m_Drive;
  public DriveCmd m_autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // set Joystick
  m_stickL = new Joystick(0);
 
  //set Joystick buttons
  fastButton = new JoystickButton(m_stickL, 3);
  slowButton = new JoystickButton(m_stickL, 4);
  limelightButton = new JoystickButton(m_stickL,5);

  //set subsystems
  m_Drive = new Drive(); 
  
  //Set default command
  m_Drive.setDefaultCommand(new DriveCmd());

  //set autoCommand
  //it is just for test
  m_autoCommand = new DriveCmd();

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
    limelightButton.whenHeld(new LimelightHold());
    limelightButton.whenReleased(new LimelightRelease());
    

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
