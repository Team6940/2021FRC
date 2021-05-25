// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.colorsensor.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Constants;
import frc.robot.util.RobotContainer;

public class turnpanel extends CommandBase {
  Color curColor;  /*当前的颜色*/ 
  Color lastColor; /*上一次的颜色*/
  int firstFlag = 0; /* 示范第一次检查 */
  int sumColors = 0; /*变化的颜色数目*/
  int sumCycles = 0;  /*转的圈数*/ 
  boolean stopMatchColorCmd =false;
  /** Creates a new turnpanel. */
  public turnpanel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_colorsensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstFlag = 0;
    sumColors = 0;
    sumCycles = 0;
    stopMatchColorCmd  = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_colorsensor.putcolor();
    RobotContainer.m_colorsensor.MotorWithPower(Constants.colorsensor.turner_power);
    boolean changeColor = false;
    curColor = RobotContainer.m_colorsensor.MatchCurrentColor();
    if( firstFlag == 0){
      lastColor = curColor;
      firstFlag = 1;
    }
    else{
      changeColor = RobotContainer.m_colorsensor.isColorChanged(lastColor);
      if(changeColor){
        lastColor = curColor;
        sumColors = sumColors+1;
        if(sumColors == 8){
          sumCycles = sumCycles + 1;
          sumColors = 0;
        }
        if(sumCycles == 3){
          RobotContainer.m_colorsensor.StopMotor();
          stopMatchColorCmd = true;
        }
      }
    }
    SmartDashboard.putNumber("sumColors", sumColors);
    SmartDashboard.putNumber("sumCycles", sumCycles);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopMatchColorCmd;
  }
}
