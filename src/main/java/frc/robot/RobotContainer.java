/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionLights;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick driveStick = new Joystick(0);
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final Drivetrain drivetrain = new Drivetrain();
  private final VisionLights visionLights = new VisionLights();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("Gyro", gyro);
    SmartDashboard.putData("Drivetrain", drivetrain);
    SmartDashboard.putData("Swerve Test",
            new SwerveTest(
                    drivetrain,
                    () -> driveStick.getX(GenericHID.Hand.kRight),
                    () -> driveStick.getY(GenericHID.Hand.kRight),
                    () -> driveStick.getZ(),
                    () -> gyro.getAngle()
            ));

    drivetrain.setDefaultCommand(
            //Allows the swerve drive command to access the joystick inputs
            new SwerveDrive(
                    drivetrain,
                    () -> driveStick.getX(GenericHID.Hand.kRight),
                    () -> driveStick.getY(GenericHID.Hand.kRight),
                    () -> driveStick.getZ(),
                    () -> gyro.getAngle()
            ));
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
