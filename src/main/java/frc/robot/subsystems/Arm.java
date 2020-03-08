/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /**
   * Creates a new Arm.
   */

  private CANSparkMax leftArm;
  private CANSparkMax rightArm;

  private CANEncoder leftArmEncoder;
  private CANEncoder rightArmEncoder;


  public Arm() {

    leftArm = new CANSparkMax(Constants.Arm_Constants.leftArmSpark_ID, MotorType.kBrushless);
    rightArm = new CANSparkMax(Constants.Arm_Constants.rightArmSpark_ID, MotorType.kBrushless);

    leftArmEncoder = new CANEncoder(leftArm);
    rightArmEncoder = new CANEncoder(rightArm);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
