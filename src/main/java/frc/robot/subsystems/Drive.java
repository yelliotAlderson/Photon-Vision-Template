// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants;

public class Drive extends SubsystemBase {
  //create variables for use
  CANSparkMax leftMaster;
  CANSparkMax leftSlave;
  CANSparkMax rightMaster;
  CANSparkMax rightSlave;
  //MotorControllerGroup leftGroup;
 // MotorControllerGroup rightGroup;
  //DifferentialDrive drivetrain;
  CommandXboxController controller;
  /** Creates a new Drive. */
  RelativeEncoder leftMasterEncoder;
  RelativeEncoder leftSlaveEncoder;
  RelativeEncoder rightMasterEncoder;
  RelativeEncoder rightSlaveEncoder;
  SparkMaxPIDController rightMasterPIDController;
  SparkMaxPIDController rightSlavePIDController;
  SparkMaxPIDController leftMasterPIDController;
  SparkMaxPIDController leftSlavePIDController;
  public boolean useJoystick;

  private static Drive instance = null;

  public static Drive getInstance() {
    if(instance == null)
      instance = new Drive();
    return instance;
  }

  public Drive() {
    //create variables for use for motors
    leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    leftSlave = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    rightSlave = new CANSparkMax(4, MotorType.kBrushless);

    //create variables for use for tracking motor position (encoder)
    leftMasterEncoder = leftMaster.getEncoder();
    leftSlaveEncoder = leftSlave.getEncoder();
    rightMasterEncoder = rightMaster.getEncoder();
    rightSlaveEncoder = rightSlave.getEncoder();

    controller = RobotContainer.getDriver();
    useJoystick = true;

    //
    rightMasterPIDController = rightMaster.getPIDController();
    rightSlavePIDController = rightSlave.getPIDController();
    leftMasterPIDController = leftMaster.getPIDController();
    leftSlavePIDController = leftSlave.getPIDController();

    configureMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (useJoystick) {
      double rightTrigger = controller.getLeftTriggerAxis();
      double leftTrigger = controller.getRightTriggerAxis();
      double leftJoystick = controller.getLeftX();
      double speed = MathUtil.applyDeadband(rightTrigger - leftTrigger, 0.1);
      double steering = MathUtil.applyDeadband(-leftJoystick, 0.1);

      steering *= robot.constants.turnSpeedInhibitor;
      speed *= robot.constants.speedInhibitor;

      double leftSpeed = speed - steering;
      double rightSpeed = speed + steering;

      leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
      rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);

      leftMaster.set(leftSpeed);
      leftSlave.set(leftSpeed);


      rightMaster.set(rightSpeed);
      rightSlave.set(rightSpeed);




    }
    SmartDashboard.putNumber("Left Master Current (Amps)", leftMaster.getOutputCurrent());
    SmartDashboard.putNumber("Left Slave Current (Amps)", leftSlave.getOutputCurrent());
    SmartDashboard.putNumber("Right Master Current (Amps)", rightMaster.getOutputCurrent());
    SmartDashboard.putNumber("Right Slave Current (Amps)", rightSlave.getOutputCurrent());
    SmartDashboard.putBoolean("Operator Control", useJoystick);
    SmartDashboard.putNumber("Left Master Current", leftMaster.getAppliedOutput());
    SmartDashboard.putNumber("leftMaster Encoder Position", leftMasterEncoder.getPosition());
    SmartDashboard.putNumber("rightMaster Encoder Position", rightMasterEncoder.getPosition());

  }

  public void zeroEncoders() {
    leftMasterEncoder.setPosition(0);
    leftSlaveEncoder.setPosition(0);
    rightMasterEncoder.setPosition(0);
    rightSlaveEncoder.setPosition(0);

  }

  public void stopdrivetrain() {
    leftMaster.set(0);
    leftSlave.set(0);
    rightMaster.set(0);
    rightSlave.set(0);

    // drivetrain.arcadeDrive(0, 0);
  }

  public void configureMotors() {
    leftMaster.setInverted(true ^ constants.driveInverted);
    leftMaster.setOpenLoopRampRate(1);
    leftMaster.setSmartCurrentLimit(40);

    leftSlave.setInverted(true ^ constants.driveInverted);
    leftSlave.setOpenLoopRampRate(1);
    leftSlave.setSmartCurrentLimit(40);

    rightMaster.setInverted(false ^ constants.driveInverted);
    rightMaster.setOpenLoopRampRate(1);
    rightMaster.setSmartCurrentLimit(40);

    rightSlave.setInverted(false ^ constants.driveInverted);
    rightSlave.setOpenLoopRampRate(1);
    rightSlave.setSmartCurrentLimit(40);

    configurePID(leftMasterPIDController);
    configurePID(leftSlavePIDController);
    configurePID(rightMasterPIDController);
    configurePID(rightSlavePIDController);

    
  }

  public void configurePID(SparkMaxPIDController pidController) {
    pidController.setP(0.01);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(0);
    pidController.setIZone(0);
    pidController.setOutputRange(-1, 1);
  }


  public double getLeftMasterEncoderPosition() {

    return leftMasterEncoder.getPosition();
  }

  public double getRightMasterEncoderPosition() {
    return rightMasterEncoder.getPosition();
  }

  public void disableDriverControl() {
    this.useJoystick = false;
    // drivetrain.stopMotor();

  }

  public void enableDriverControl() {
    this.useJoystick = true;
  }

  public void setLeftTarget(double target) {
    SmartDashboard.putNumber("Left Target", target);
    leftMasterPIDController.setReference(target, ControlType.kPosition);
    leftSlavePIDController.setReference(target, ControlType.kPosition);

  }

  public void setRightTarget(double target) {
    SmartDashboard.putNumber("Right Target", target);
    rightMasterPIDController.setReference(target, ControlType.kPosition);
    rightSlavePIDController.setReference(target, ControlType.kPosition);

  }

  public void setLeftSpeed(double speed) {
    leftMaster.set(speed);
    leftSlave.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightMaster.set(speed);
    rightSlave.set(speed);
  }

  public void setDrivetrainTarget() {

  }
}
