// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {
  /** Creates a new vision. */
  public UsbCamera m_usbcamerafront; //= new UsbCamera("m_usbcamera1", 0);
  public UsbCamera m_usbcameraside; //= new UsbCamera("m_usbcmaera2", 1);
  public vision() {
    m_usbcamerafront = new UsbCamera("m_usbcamera1", 0);
    m_usbcameraside = new UsbCamera("m_usbcmaera2", 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
