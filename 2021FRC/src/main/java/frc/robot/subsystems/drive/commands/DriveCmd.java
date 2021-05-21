// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;
import frc.robot.util.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;

/** An example command that uses an example subsystem. */
public class DriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 
  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmd(){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //todo
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //todo
    double x =0;
    double z = 0;
    x = RobotContainer.m_joystick.getY(GenericHID.Hand.kRight);
    z = RobotContainer.m_joystick.getX(GenericHID.Hand.kRight);
    RobotContainer.m_Drive.DriveCar(x, z);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //todo
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
