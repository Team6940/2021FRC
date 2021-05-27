// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;
import frc.robot.util.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCmd extends CommandBase {
  /** Creates a new Shoot. */
  public ShootCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double a = 0;
    a = RobotContainer.m_operatorjoystick.getTriggerAxis(GenericHID.Hand.kRight);
    SmartDashboard.putNumber("a", a);
    if(a > 0){
      RobotContainer.m_Shooter.m_shooter_switch = true;
    }
    else{
      RobotContainer.m_Shooter.m_shooter_switch = false;
    }
    RobotContainer.m_Shooter.OutputShooter();
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
