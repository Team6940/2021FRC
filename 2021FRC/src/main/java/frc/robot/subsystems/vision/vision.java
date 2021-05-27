// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {
  /** Creates a new vision. */
  public UsbCamera m_usbcamerafront; //= new UsbCamera("m_usbcamera1", 0);
  public UsbCamera m_usbcameraside; //= new UsbCamera("m_usbcmaera2", 1);
  public vision() {
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);
    /*new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
