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
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class cameraSystem extends SubsystemBase {
  /** Creates a new cameraSystem. */

  AprilTagFieldLayout aprilTagFieldLayout;

  PhotonCamera cam = new PhotonCamera("testCamera");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Cam mounted
                                                                                                       // facing
                                                                                                       // forward, half
                                                                                                       // a meter
                                                                                                       // forward of
                                                                                                       // center, half a
                                                                                                       // meter up from
                                                                                                       // center.
  RobotPoseEstimator robotPoseEstimator;
  UsbCamera frontCamera;
//  UsbCamera reverseCamera;
  MjpegServer view;

  private static cameraSystem m_CameraSystem = new cameraSystem();
  // ... Add other cameras here

  // Assemble the list of cameras & mount locations
  ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

  public static cameraSystem getCameraSystem() {

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
    try {
      frontCamera = CameraServer.startAutomaticCapture();
      frontCamera.setResolution(50, 50);
      // reverseCamera = CameraServer.startAutomaticCapture(1);
      // reverseCamera.setResolution(50, 50);
      // reverseCamera.setPixelFormat(PixelFormat.kMJPEG);

      frontCamera.setFPS(10);
      // reverseCamera.setFPS(10);

      // frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      // reverseCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

      view = new MjpegServer("primary view", "", 8008);
      view.setSource(frontCamera);

      CameraServer.addServer(view);
    } catch (Exception e) {
      // TODO: handle exception
    }

  }

  public void changeCamera(beanieController controller) {
    try {
      if (controller.getLeftYAxis() >= 0) {
        // System.out.println("Setting camera 2");
        // view.setSource(frontCamera);
      }
      else;

      // } else if (controller.getLeftYAxis() < 0) {
      //   // System.out.println("Setting camera 1");
      //   view.setSource(reverseCamera);

      }
     catch (Exception e) {
      // TODO: handle exception
    }

  }

  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    if(DriverStation.getAlliance() == Alliance.Red){
      aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }
    else if(DriverStation.getAlliance() == Alliance.Blue){
      aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

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
