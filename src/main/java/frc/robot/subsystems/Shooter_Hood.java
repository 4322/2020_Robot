/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_Hood extends SubsystemBase {
  /**
   * Creates a new Shooter_Hood.
   */

   private WPI_TalonSRX hoodMotor;
   private Encoder hoodEncoder;

  public Shooter_Hood() {

    hoodMotor = new WPI_TalonSRX(Constants.Shooter_Constants.hoodTalon_ID);

    hoodEncoder = new Encoder(0, 1);

  }

  public void setHood(double power)
  {
    hoodMotor.set(power);
  }

  public void resetEncoder()
  {
    hoodEncoder.reset();
  }

  public double getEncoder_Velocity()
  {
    return hoodEncoder.getRate();
  }

  public double getEncoder_Position()
  {
    return hoodEncoder.getDistance();
  }



  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
