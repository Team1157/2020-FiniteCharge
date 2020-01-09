/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;


/**
 * Point the wheels straight forwards and drives like a standard differential drive.
 */
public class SwerveDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain drivetrain;

  private DoubleSupplier joystickX;
  private DoubleSupplier joystickY;
  private DoubleSupplier joystickZ;

  /**
   * Creates a new SwerveDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDrive(Drivetrain subsystem, DoubleSupplier getJoystickX, DoubleSupplier getJoystickY, DoubleSupplier getJoystickZ) {
    drivetrain = subsystem;
    joystickX = getJoystickX;
    joystickY = getJoystickY;
    joystickZ = getJoystickZ;

    //Declare dependency on the drivetrain subsystem
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
