/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivebase extends PIDSubsystem {

  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave1;
  private CANSparkMax rightSlave2;

  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave1;
  private CANSparkMax leftSlave2;

  private CANEncoder rightMaster_encoder;
  private CANEncoder rightSlave1_encoder;
  private CANEncoder rightSlave2_encoder;

  private CANEncoder leftMaster_encoder;
  private CANEncoder leftSlave1_encoder;
  private CANEncoder leftSlave2_encoder;

  private SpeedControllerGroup rightMotors;
  private SpeedControllerGroup leftMotors;


  private DifferentialDrive drive;

  /**
   * Creates a new Drivebase.
   */
  public Drivebase() {

    super(new PIDController(Constants.Drivebase_Constants.PID_Values.kP, Constants.Drivebase_Constants.PID_Values.kI, Constants.Drivebase_Constants.PID_Values.kD));
    
    rightMaster = new CANSparkMax(Constants.Drivebase_Constants.rightMasterSpark_ID, MotorType.kBrushless);
    rightSlave1 = new CANSparkMax(Constants.Drivebase_Constants.rightSlave1Spark_ID, MotorType.kBrushless);
    rightSlave2 = new CANSparkMax(Constants.Drivebase_Constants.rightSlave2Spark_ID, MotorType.kBrushless);

    leftMaster = new CANSparkMax(Constants.Drivebase_Constants.leftMasterSpark_ID, MotorType.kBrushless);
    leftSlave1 = new CANSparkMax(Constants.Drivebase_Constants.leftSlave1Spark_ID, MotorType.kBrushless);
    leftSlave2 = new CANSparkMax(Constants.Drivebase_Constants.leftSlave2Spark_ID, MotorType.kBrushless);

    rightMaster_encoder = rightMaster.getEncoder();
    rightSlave1_encoder = rightSlave1.getEncoder();
    rightSlave2_encoder = rightSlave2.getEncoder();

    leftMaster_encoder = leftMaster.getEncoder();
    leftSlave1_encoder = leftSlave1.getEncoder();
    leftSlave2_encoder = leftSlave2.getEncoder();

    rightMaster.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);
    rightSlave1.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);
    rightSlave2.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);

    leftMaster.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);
    leftSlave1.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);
    leftSlave2.setSmartCurrentLimit(Constants.Drivebase_Constants.SparkMax_CurrentLimit);

    rightMaster.burnFlash();
    rightSlave1.burnFlash();
    rightSlave2.burnFlash();

    leftMaster.burnFlash();
    leftSlave1.burnFlash();
    leftSlave2.burnFlash();

    rightMotors = new SpeedControllerGroup(rightMaster, rightSlave1, rightSlave2);
    leftMotors = new SpeedControllerGroup(leftMaster, leftSlave1, leftSlave2);

    drive = new DifferentialDrive(leftMotors, rightMotors);

  }

  
  /****************************************************
   ************ DIFFERENTIAL DRIVE MODES **************
   ****************************************************/
  public void curveDrive(double power, double turn, boolean quickTurn)
  {
    drive.curvatureDrive(power, turn, quickTurn);
  }

  public void arcadeDrive(double power, double turn, boolean squaredInputs)
  {
    drive.arcadeDrive(power, turn, squaredInputs);
  }


  /****************************************************
   ********* GETTING GROUPED ENCODER VALUES ***********
   ************* (POSITION / VELOCITY) ****************
   ****************************************************/
  public double getRightEncoders_Position()
  {
    return ((leftMaster_encoder.getPosition() + leftSlave1_encoder.getPosition() + leftSlave2_encoder.getPosition())/3);
  }

  public double getLeftEncoders_Position()
  {
    return ((rightMaster_encoder.getPosition() + rightSlave1_encoder.getPosition() + rightSlave2_encoder.getPosition())/3);
  }

  public double getRightEncoders_Velocity()
  {
    return ((rightMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity() + rightSlave2_encoder.getVelocity())/3);
  }

  public double getLeftEncoders_Velocity()
  {
    return ((leftMaster_encoder.getVelocity() + rightSlave1_encoder.getVelocity() + rightSlave2_encoder.getVelocity())/3);
  }

  /****************************************************
   ********* GETTING SINGLE ENCODER VALUES ************
   ****************** (POSITION) **********************
   ****************************************************/
   public double getRightMasterEncoderPosition()
   {
      return rightMaster_encoder.getPosition();
   }

   public double getRightSlave1EncoderPosition()
   {
      return rightSlave1_encoder.getPosition();
   }

   public double getRightSlave2EncoderPosition()
   {
      return rightSlave2_encoder.getPosition();
   }

   public double getLeftMasterEncoderPosition()
   {
      return leftMaster_encoder.getPosition();
   }

   public double getLeftSlave1EncoderPosition()
   {
      return leftSlave1_encoder.getPosition();
   }

   public double getLeftSlave2EncoderPosition()
   {
      return leftSlave2_encoder.getPosition();
   }

  /****************************************************
   ********* GETTING SINGLE ENCODER VALUES ************
   ****************** (VELOCITY) **********************
   ****************************************************/
   public double getRightMasterEncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }
   
   public double getRightSlave1EncoderVelocity()
   {
      return rightSlave1_encoder.getVelocity();
   }
   
   public double getRightSlave2EncoderVelocity()
   {
      return rightSlave2_encoder.getVelocity();
   }

   public double getLeftMasterEncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }

   public double getLeftSlave1EncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }

   public double getLeftSlave2EncoderVelocity()
   {
      return rightMaster_encoder.getVelocity();
   }

  /****************************************************
   ** DISPLAYING GROUPED ENCODER VALUES ON DASHBOARD **
   ************* (POSITION / VELOCITY) ****************
   ****************************************************/
  public void displayLeftEncodersPosition()
  {
    SmartDashboard.putNumber("Left Side Encoder", getLeftEncoders_Position());
  }

  public void displayRightEncodersPosition()
  {
    SmartDashboard.putNumber("Right Side Encoder Position", getRightEncoders_Position());
  }

  public void displayLeftEncodersVelocity()
  {
    SmartDashboard.putNumber("Left Side Encoder Velocity", getLeftEncoders_Velocity());
  }

  public void displayRightEncodersVelocity()
  {
    SmartDashboard.putNumber("Right Side Encoder Velocity", getRightEncoders_Velocity());
  }

  /*****************************************************
   * DISPLAYING INDIVIDUAL ENCODER VALUES ON DASHBOARD *
   ************* (POSITION / VELOCITY) *****************
   *****************************************************/
  public void displayAllLeftSideEncoders_Position()
  {
    double[] leftEncoders = new double[]{getLeftMasterEncoderPosition(), getLeftSlave1EncoderPosition(), getLeftSlave2EncoderPosition()};
    SmartDashboard.putNumberArray("Left Encoder Position (Master, Slave1, Slave2)", leftEncoders);
  }

  public void displayAllRightSideEncoders_Position()
  {
    double[] rightEncoders = new double[]{getRightMasterEncoderPosition(), getRightSlave1EncoderPosition(), getRightSlave2EncoderPosition()};
    SmartDashboard.putNumberArray("Right Encoder Position (Master, Slave1, Slave2)", rightEncoders);
  }

  public void displayAllRightSideEncoders_Velocity()
  {
    double[] rightEncoders = new double[]{getRightMasterEncoderVelocity(), getRightSlave1EncoderVelocity(), getRightSlave2EncoderVelocity()};
    SmartDashboard.putNumberArray("Right Encoder Velocity (Master, Slave1, Slave2)", rightEncoders);
  }

  public void displayAllLeftSideEncoders_Velocity()
  {
    double[] leftEncoders = new double[]{getLeftMasterEncoderVelocity(), getLeftSlave1EncoderVelocity(), getLeftSlave2EncoderVelocity()};
    SmartDashboard.putNumberArray("Left Encoder Velocity (Master, Slave1, Slave2)", leftEncoders);
  }

  /****************************************************
   * SETTING POSITION AND VELOCITY CONVERSION FACTORS *
   ****************************************************/
  public void setPositionConversionFactor()
  {
    rightMaster_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave1_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave2_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);

    leftMaster_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    leftSlave1_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    leftSlave2_encoder.setPositionConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
  }

  public void setVelocityConversionFactor()
  {
    rightMaster_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave1_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave2_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);

    rightMaster_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave1_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
    rightSlave2_encoder.setVelocityConversionFactor(Constants.Drivebase_Constants.positionConversionFactor);
  }
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub

  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
