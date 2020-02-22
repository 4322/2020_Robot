/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");


  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getX_Offset()
  {
    return tx.getDouble(0);
  }

  public double getY_Offset()
  {
    return ty.getDouble(0);
  }

  public double get_TargetArea()
  {
    return ta.getDouble(0);
  }

  public boolean getTarget()
  {
    return tv.getBoolean(false);
  }

  public void displayValues()
  {
    SmartDashboard.putNumber("X Offset", getX_Offset());
    SmartDashboard.putNumber("Y Offset", getY_Offset());
    SmartDashboard.putBoolean("Target Visible", getTarget());
    SmartDashboard.putNumber("Target Area", get_TargetArea());
  }




}
