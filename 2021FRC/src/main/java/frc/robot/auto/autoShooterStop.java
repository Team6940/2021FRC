// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.RobotContainer;
import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class autoShooterStop extends CommandBase {
  int ticks = 0;
  int delayseconds = 0;
  public autoShooterStop(int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    delayseconds = seconds;
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticks = 0;
    //RobotContainer.m_intake.m_intake_switch = false;
    //RobotContainer.m_intake.OutputIntake();
    RobotContainer.m_BallTrans.m_balltrans_switch = false;
    RobotContainer.m_BallTrans.OutputBalltrans();
    RobotContainer.m_Shooter.m_shooter_switch = false;
    RobotContainer.m_Shooter.OutputShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticks++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       return true;
  }
}
