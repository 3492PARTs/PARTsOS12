// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import PARTSlib2023.PARTS.frc.Utils.Controls.beanieController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class cameraSystem extends SubsystemBase {
  /** Creates a new cameraSystem. */

  AprilTagFieldLayout aprilTagFieldLayout;
    
  PhotonCamera cam = new PhotonCamera("testCamera");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  RobotPoseEstimator robotPoseEstimator;
  UsbCamera frontCamera;
  UsbCamera reverseCamera;
  VideoSink server;

  

  private static cameraSystem m_CameraSystem = new cameraSystem();
    // ... Add other cameras here

    // Assemble the list of cameras & mount locations
  ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

  public static cameraSystem getCameraSystem(){

    return m_CameraSystem;
  }
    
  
  
  public cameraSystem() {
    camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.kDefaultField.m_resourceFile);
    
      } catch (Exception e) {
        // TODO: handle exception
      }
      robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camList);

      frontCamera = CameraServer.startAutomaticCapture(0);
      frontCamera.setResolution(300, 300);
      reverseCamera = CameraServer.startAutomaticCapture(1);
      reverseCamera.setResolution(300, 300);

      frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      reverseCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

      server = CameraServer.getServer();

  

  }

  public void changeCamera(beanieController controller){
    if (controller.getLeftYAxis() >= 0) {
      // System.out.println("Setting camera 2");
      server.setSource(frontCamera);
  } else if (controller.getLeftYAxis() < 0) {
      // System.out.println("Setting camera 1");
      server.setSource(reverseCamera);
  }
  }

  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
