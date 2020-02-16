/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave1;
  private CANSparkMax rightSlave2;

  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave1;
  private CANSparkMax leftSlave2;

  private SpeedControllerGroup rightMotors;
  private SpeedControllerGroup leftMotors;

  private DifferentialDrive drive;

  /**
   * Creates a new Drivebase.
   */
  public Drivebase() {

    rightMaster = new CANSparkMax(Constants.Drivebase_Constants.rightMasterSpark_ID, MotorType.kBrushless);
    rightSlave1 = new CANSparkMax(Constants.Drivebase_Constants.rightSlave1Spark_ID, MotorType.kBrushless);
    rightSlave2 = new CANSparkMax(Constants.Drivebase_Constants.rightSlave2Spark_ID, MotorType.kBrushless);

    leftMaster = new CANSparkMax(Constants.Drivebase_Constants.leftMasterSpark_ID, MotorType.kBrushless);
    leftSlave1 = new CANSparkMax(Constants.Drivebase_Constants.leftSlave1Spark_ID, MotorType.kBrushless);
    leftSlave2 = new CANSparkMax(Constants.Drivebase_Constants.leftSlave2Spark_ID, MotorType.kBrushless);

    rightMaster.setSmartCurrentLimit(40);

    rightMotors = new SpeedControllerGroup(rightMaster, rightSlave1, rightSlave2);
    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave1, leftSlave2);

    drive = new DifferentialDrive(leftMotors, rightMotors);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
