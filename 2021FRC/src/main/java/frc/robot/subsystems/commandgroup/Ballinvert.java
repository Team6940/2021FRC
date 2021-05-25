// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.commandgroup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;

public class Ballinvert extends CommandBase {
  /** Creates a new Ballinvert. */
  double time_start;
  double time_end;
  public Ballinvert() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_BallTrans);
    addRequirements(RobotContainer.m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time_start = RobotContainer.m_timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time_cur = RobotContainer.m_timer.get();
    RobotContainer.m_Shooter.invertShooter();
    if(time_cur - time_start > 1){
      RobotContainer.m_BallTrans.invertballtrans();
    }
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
