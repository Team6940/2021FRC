// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;

public class LimelightReleaseCmd extends CommandBase {
  /** Creates a new LimelightReleaseCmd. */
  public LimelightReleaseCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Drive.setLightMode(Constants.Limelight.LED_OFF);
    SmartDashboard.putNumber("LED", Constants.Limelight.LED_OFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Drive.auto = false;
    SmartDashboard.putNumber("auto", 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
