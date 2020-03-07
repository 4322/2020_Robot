/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Shooter extends PIDSubsystem {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax flywheelOne;
  private CANSparkMax flywheelTwo;
  private CANSparkMax kickerMotor;
  private WPI_TalonSRX shooterHood;

  private CANEncoder flywheelOne_Encoder;
  private CANEncoder flywheelTwo_Encoder;
  private CANEncoder kickerMotor_Encoder;

  private Encoder shooterHood_Encoder;




  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Shooter_Constants.PID_Values.kP, Constants.Shooter_Constants.PID_Values.kI, Constants.Shooter_Constants.PID_Values.kD));
    
    flywheelOne = new CANSparkMax(Constants.Shooter_Constants.flywheelOneSpark_ID, MotorType.kBrushless);
    flywheelTwo = new CANSparkMax(Constants.Shooter_Constants.flywheelTwoSpark_ID, MotorType.kBrushless);
    kickerMotor = new CANSparkMax(Constants.Shooter_Constants.kickerSpark_ID, MotorType.kBrushless);
    shooterHood = new WPI_TalonSRX(Constants.Shooter_Constants.hoodTalon_ID);

    flywheelOne_Encoder = new CANEncoder(flywheelOne);
    flywheelTwo_Encoder = new CANEncoder(flywheelTwo);
    kickerMotor_Encoder = new CANEncoder(kickerMotor);

    flywheelOne.setIdleMode(IdleMode.kCoast);
    flywheelTwo.setIdleMode(IdleMode.kCoast);

    flywheelOne.burnFlash();
    flywheelTwo.burnFlash();

    shooterHood_Encoder = new Encoder(0, 1);  //USES DIO PINS 0 AND 1 ON THE ROBORIO

    flywheelOne.setInverted(true);
    
    flywheelTwo.follow(flywheelOne, true);

    
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();

    displayEncoderValues();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void stopShooter()
  {
    flywheelOne.set(0);
  }

  public void stopKicker()
  {
    flywheelTwo.set(0);
  }

  public void spinShooter()
  {
    flywheelOne.set(.5);
  }

  public void spinKicker()
  {
    kickerMotor.set(.5);
  }

  public void setHood(double power)
  {
    shooterHood.set(power);
  }

  public double getShooterEncoder_Position()
  {
    return ((flywheelOne_Encoder.getPosition() + flywheelTwo_Encoder.getPosition()) / 2);
  }

  public double getShooterEncoder_Velocity()
  {
    return ((flywheelOne_Encoder.getVelocity() + flywheelTwo_Encoder.getVelocity() / 2));
  }

  public double getKickerEncoder_Position()
  {
    return kickerMotor_Encoder.getPosition();
  }

  public double getKickerEncoder_Velocity()
  {
    return kickerMotor_Encoder.getVelocity();
  }

  public double getHoodEncoder_Position()
  {
    return shooterHood_Encoder.getDistance();
  }

  public void displayEncoderValues()
  {
    SmartDashboard.putNumber("Shooter Velocity", getShooterEncoder_Velocity());
    SmartDashboard.putNumber("Shooter Position", getShooterEncoder_Position());
    SmartDashboard.putNumber("Kicker Velocity", getKickerEncoder_Velocity());
    SmartDashboard.putNumber("Kicker Position", getKickerEncoder_Position());
    SmartDashboard.putNumber("Hood Position", getHoodEncoder_Position());
  }
}
