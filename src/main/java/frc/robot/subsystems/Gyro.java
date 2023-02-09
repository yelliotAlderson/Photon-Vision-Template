// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Method;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;

public class Gyro extends SubsystemBase {
    public Pigeon2 Pigeon;
    Pigeon2Configuration PigeonConfig = new Pigeon2Configuration();


    double Roll;
    double Pitch;
    double Yaw;
    
    Pigeon = new Pigeon2(1);
    public void Gyro() {
        Yaw = Pigeon.getYaw();
        Pitch = Pigeon.getPitch();
        Roll = Pigeon.getRoll();

    }
    
    @Override
   
    public void periodic() {
        Gyro();
        SmartDashboard.putNumber("Yaw: ", Yaw);
        SmartDashboard.putNumber("Pitch: ", Pitch);
        SmartDashboard.putNumber("Roll: ", Roll);

    }

}
