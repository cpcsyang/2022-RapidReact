// Dummy subsystem class

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverVisionSubsystem extends SubsystemBase {

  UsbCamera camera;
  String cameraName;
  VideoSink server;

  FeederSubsystem feeder;
  boolean readyToShoot;

  public DriverVisionSubsystem(FeederSubsystem feeder) {
    camera();

    this.feeder = feeder;
    shoot();
    
  }

  public void camera() {
    camera = CameraServer.startAutomaticCapture("cameraA", 0);
    cameraName = camera.getName();

    server = CameraServer.getServer();
    server.setSource(camera);

    Shuffleboard.getTab("Driver")
                .add("Camera", server.getSource())
                .withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 5)
                .withPosition(0, 0);
  }

  public void shoot() {
    readyToShoot = feeder.ballInExit();

    Shuffleboard.getTab("Driver")
                .add("Read to Shoot", readyToShoot)
                .withSize(1, 1)
                .withPosition(6, 0);
  }

  @Override
  public void periodic() {}
}