/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Collecter_Collect;
import frc.robot.commands.Collector_Eject;
import frc.robot.commands.Collector_Stop;
import frc.robot.commands.Disable_Kicker;
import frc.robot.commands.Disable_Shooter;
import frc.robot.commands.Drive_Manual;
import frc.robot.commands.Enable_Kicker;
import frc.robot.commands.Enable_Shooter;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Hood_Manual;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter_Hood;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Limelight limelight = new Limelight();
  public final Drivebase drivebase = new Drivebase();
  public final Shooter shooter = new Shooter();
  public final Shooter_Hood shooterHood = new Shooter_Hood();
  public final Kicker kicker = new Kicker();
  public final Collector collector = new Collector();
  

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final Drive_Manual driveManual = new Drive_Manual(drivebase);

  public final Hood_Manual hoodManual = new Hood_Manual(shooterHood);

  public final Enable_Shooter enableShooter = new Enable_Shooter(shooter);
  public final Disable_Shooter disableShooter = new Disable_Shooter(shooter);

  public final Enable_Kicker enableKicker = new Enable_Kicker(kicker);
  public final Disable_Kicker disableKicker = new Disable_Kicker(kicker);

  public final Collecter_Collect collectorCollect = new Collecter_Collect(collector);
  public final Collector_Eject collectorEject = new Collector_Eject(collector);
  public final Collector_Stop collectorStop = new Collector_Stop(collector);
  

  public static frc.robot.XboxController pilot = new frc.robot.XboxController(0);
  public static frc.robot.XboxController coPilot = new frc.robot.XboxController(1);

  


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivebase.setDefaultCommand(driveManual);
    shooterHood.setDefaultCommand(hoodManual);
    collector.setDefaultCommand(collectorStop);


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    coPilot.lb.whenPressed(enableShooter);
    coPilot.rb.whenPressed(disableShooter);

    coPilot.x.whenPressed(enableKicker);
    coPilot.b.whenPressed(disableKicker);

    pilot.lt.whileHeld(collectorCollect, true);
    pilot.rt.whileHeld(collectorEject, true);

    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
