// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.balltrans.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;

public class InvertBall extends CommandBase {
  /** Creates a new InvertBall. */
  public double time_end;
  public InvertBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_BallTrans);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time_end = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_BallTrans.invertballtrans();
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
