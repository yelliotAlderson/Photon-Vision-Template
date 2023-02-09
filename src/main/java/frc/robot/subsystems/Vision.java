// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.LambdaConversionException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public PhotonCamera camera;
  public PhotonPipelineResult lastResult;

  /** Creates a new Vision. */
  public Vision() {
    camera = new PhotonCamera("OV5647");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lastResult = camera.getLatestResult();
    
    // 
    SmartDashboard.putNumber("Number of targets", lastResult.getTargets().size());
    SmartDashboard.putNumber("Index", camera.getPipelineIndex());

    if (lastResult.hasTargets()) {
      SmartDashboard.putString("Best Target", lastResult.getBestTarget().getBestCameraToTarget().toString());
      SmartDashboard.putNumber("Target Area", lastResult.getBestTarget().getArea());
      
    }
  }
  public boolean hasTargets(){
    return lastResult.hasTargets();
  }
  public double getTargetDistance(){
    return lastResult.getBestTarget().getBestCameraToTarget().getX();
  }
  public void togglePiplines() {
    int indexNum = camera.getPipelineIndex();
    SmartDashboard.putNumber("Index(0/1)", camera.getPipelineIndex());
    if (indexNum == 1) {
      indexNum = 0;
    } else {
      indexNum = 1;
    }
    camera.setPipelineIndex(indexNum);
  }
}
