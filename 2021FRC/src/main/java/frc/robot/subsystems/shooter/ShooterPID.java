// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import frc.robot.util.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class ShooterPID extends PIDSubsystem {
  /** Creates a new ShooterPID. */
  private final WPI_TalonFX m_shooterLeftMotor = new WPI_TalonFX(ShooterConstants.kShooterLeftMotorPort);
  private final WPI_TalonFX m_shooterRghtMotor = new WPI_TalonFX(ShooterConstants.kShooterRghtMotorPort);
  private final WPI_TalonFX m_feederMotor = new WPI_TalonFX(ShooterConstants.kFeederMotorPort);
  private final Encoder m_shooterEncoder =
  new Encoder(
      ShooterConstants.kEncoderPorts[0],
      ShooterConstants.kEncoderPorts[1],
      ShooterConstants.kEncoderReversed);
  private final SimpleMotorFeedforward m_shooterFeedforward =
  new SimpleMotorFeedforward(
      ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  public ShooterPID() {
        super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
        m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
        setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterLeftMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    m_shooterRghtMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_shooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }
}
