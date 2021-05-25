// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.colorsensor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;

public class StopMotor extends CommandBase {
  /** Creates a new StopMotor. */
  public StopMotor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_colorsensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_colorsensor.StopMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
