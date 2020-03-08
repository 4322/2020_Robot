/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private CANSparkMax leftArm;
  private CANSparkMax rightArm;
  private CANEncoder leftArm_encoder;
  private CANEncoder rightArm_encoder;

  private CANPIDController ArmPidController;
  
  public Arm() {
      leftArm = new CANSparkMax(Constants.ArmConstants.leftMotor_ID, MotorType.kBrushless);
      rightArm = new CANSparkMax(Constants.ArmConstants.rightMotor_ID, MotorType.kBrushless);



      leftArm.follow(rightArm, true);

      ArmPidController.setP(Constants.ArmConstants.PID_Values.kP);
      ArmPidController.setI(Constants.ArmConstants.PID_Values.kI);
      ArmPidController.setD(Constants.ArmConstants.PID_Values.kP);
      ArmPidController.setIZone(Constants.ArmConstants.PID_Values.kIz);
      ArmPidController.setFF(Constants.ArmConstants.PID_Values.kFF);
      ArmPidController.setOutputRange(Constants.ArmConstants.PID_Values.kMinOutput,Constants.ArmConstants.PID_Values.kMaxOutput);

      int smartmotionslot = 0;
      ArmPidController.setSmartMotionMaxVelocity(Constants.ArmConstants.PID_Values.maxVelocity, smartmotionslot);
      ArmPidController.setSmartMotionMinOutputVelocity(Constants.ArmConstants.PID_Values.minVelocity, smartmotionslot);
      ArmPidController.setSmartMotionMaxAccel(Constants.ArmConstants.PID_Values.maxAcceleration, smartmotionslot);
    
      ArmPidController.setSmartMotionAllowedClosedLoopError(Constants.ArmConstants.PID_Values.allowed_error, smartmotionslot);
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", Constants.ArmConstants.PID_Values.kP);
      SmartDashboard.putNumber("I Gain", Constants.ArmConstants.PID_Values.kI);
      SmartDashboard.putNumber("D Gain", Constants.ArmConstants.PID_Values.kD);
      SmartDashboard.putNumber("I Zone", Constants.ArmConstants.PID_Values.kIz);
      SmartDashboard.putNumber("Feed Forward", Constants.ArmConstants.PID_Values.kFF);
      SmartDashboard.putNumber("Max Output", Constants.ArmConstants.PID_Values.kMaxOutput);
      SmartDashboard.putNumber("Min Output", Constants.ArmConstants.PID_Values.kMinOutput);

      // display Smart Motion coefficients
      SmartDashboard.putNumber("Max Velocity", Constants.ArmConstants.PID_Values.maxVelocity);
      SmartDashboard.putNumber("Min Velocity", Constants.ArmConstants.PID_Values.minVelocity);
      SmartDashboard.putNumber("Max Acceleration", Constants.ArmConstants.PID_Values.maxAcceleration);
      SmartDashboard.putNumber("Allowed Closed Loop Error", Constants.ArmConstants.PID_Values.allowed_error);
      SmartDashboard.putNumber("Set Position", 0);
      SmartDashboard.putNumber("Set Velocity", 0);
  
  }

  public double getArmEncoderPosition() {
    return (leftArm_encoder.getPosition() + rightArm_encoder.getPosition())/2;
  }

  public double getArmEncoderVelocity() {
    return (leftArm_encoder.getVelocity() + leftArm_encoder.getVelocity()) /2;
  }

  public void displayArmEncoderValues() {
    SmartDashboard.putNumber("Arm Encoder Position", getArmEncoderPosition());
    SmartDashboard.putNumber("Arm Encoder Velocity", getArmEncoderVelocity());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
