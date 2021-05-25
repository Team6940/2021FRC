// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.colorsensor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  public final ColorSensorV3 m_ColorSensor;
  public final ColorMatch m_ColorMatcher;
  public WPI_TalonSRX m_colorturner;
  public Solenoid m_solenoidcolor;

  //  0 for push ,1 for back
  public int push_or_back_flag = 0;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // set color inits
  public Color detectedColor;

  public ColorSensor() {
    m_ColorSensor = Robot.hardware.m_ColorSensor;
    m_ColorMatcher = Robot.hardware.m_ColorMatcher;
    m_colorturner = Robot.hardware.m_colorturner;
    m_solenoidcolor = Robot.hardware.m_solenoidcolor;

    m_ColorMatcher.addColorMatch(kBlueTarget);
    m_ColorMatcher.addColorMatch(kGreenTarget);
    m_ColorMatcher.addColorMatch(kRedTarget);
    m_ColorMatcher.addColorMatch(kYellowTarget); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MotorWithPower(double turner_power){
    m_colorturner.set(ControlMode.PercentOutput,turner_power);
  }

  public void StopMotor(){
    m_colorturner.setNeutralMode(NeutralMode.Brake);
  }

  public void GetColor(){
    detectedColor = m_ColorSensor.getColor();
  }

  public void StopPanelWithColor(Color color){
    GetColor();
    ColorMatchResult match = m_ColorMatcher.matchClosestColor(detectedColor);
    if(match.color == color){
      StopMotor();
    }
  }

  public boolean isColorChanged(Color TargetColor){
    Color detectedColor = m_ColorSensor.getColor();
    ColorMatchResult match = m_ColorMatcher.matchClosestColor(detectedColor);
    if (match.color == TargetColor) {
      return false;
    }
    else{
      return true;
    }
  }

  public void putcolor(){
    String colorString;
    ColorMatchResult match = m_ColorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";

          /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    }

  }

  public Color MatchCurrentColor(){
    Color detectedColor = m_ColorSensor.getColor();
    ColorMatchResult match = m_ColorMatcher.matchClosestColor(detectedColor);
    return match.color;

  }

  public void MatchColor(){
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */

    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          StopPanelWithColor(kRedTarget);
          break;
        case 'G' :
          //Green case code
          StopPanelWithColor(kYellowTarget);
          break;                                                                                                                                                                                                                                                             
        case 'R' :
          //Red case code
          StopPanelWithColor(kBlueTarget);
          break;
        case 'Y' :
          //Yellow case code
          StopPanelWithColor(kGreenTarget);
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
    }
  }

  public void colorsensorsolenoid(boolean on){
    m_solenoidcolor.set(on);
  }
}
