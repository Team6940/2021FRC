// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.balltrans.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.RobotContainer;

public class BallTransCmd extends CommandBase {
  /** Creates a new BallTransCmd. */
  public BallTransCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_BallTrans);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double b;
    b = RobotContainer.m_joystick.getTriggerAxis(GenericHID.Hand.kLeft);
    if(b > 0){
      RobotContainer.m_BallTrans.m_balltrans_switch = true;
    }
    else{
      RobotContainer.m_BallTrans.m_balltrans_switch = false;
    }
    RobotContainer.m_BallTrans.OutputBalltrans();
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
