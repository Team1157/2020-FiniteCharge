/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;
import java.util.function.DoubleSupplier;


/**
 * Drives the robot utilizing the full capabilities of swerve drive
 */
public class SwerveDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Object handles the high level math for calculating wheel speeds and angles
  private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
          Drivetrain.wheelCoordinates.get(Drivetrain.MotorLocation.FRONT_LEFT),
          Drivetrain.wheelCoordinates.get(Drivetrain.MotorLocation.FRONT_RIGHT),
          Drivetrain.wheelCoordinates.get(Drivetrain.MotorLocation.BACK_LEFT),
          Drivetrain.wheelCoordinates.get(Drivetrain.MotorLocation.BACK_RIGHT)
  );

  private final Drivetrain drivetrain;

  private DoubleSupplier joystickX;
  private DoubleSupplier joystickY;
  private DoubleSupplier joystickZ;
  private DoubleSupplier gyro;

  /**
   * Creates a new SwerveDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDrive(Drivetrain subsystem, DoubleSupplier getJoystickX, DoubleSupplier getJoystickY, DoubleSupplier getJoystickZ, DoubleSupplier getGyroAngle) {
    drivetrain = subsystem;
    joystickX = getJoystickX;
    joystickY = getJoystickY;
    joystickZ = getJoystickZ;
    gyro = getGyroAngle;

    //Declare dependency on the drivetrain subsystem
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Create a WPILib object representing the desired velocity of the robot
    ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            joystickY.getAsDouble() * 2.0,
            joystickX.getAsDouble() * -2.0,
            joystickZ.getAsDouble() * Math.PI / 2.0,
            Rotation2d.fromDegrees(gyro.getAsDouble())
    );

    //Calculates the desired state for each swerve module
    SwerveModuleState[] moduleStatesList = swerveDriveKinematics.toSwerveModuleStates(desiredSpeeds);

    Map<Drivetrain.MotorLocation, SwerveModuleState> moduleStates = Map.of(
            Drivetrain.MotorLocation.FRONT_LEFT, moduleStatesList[0],
            Drivetrain.MotorLocation.FRONT_RIGHT, moduleStatesList[1],
            Drivetrain.MotorLocation.BACK_LEFT, moduleStatesList[2],
            Drivetrain.MotorLocation.BACK_RIGHT, moduleStatesList[3]
    );

    //Set the drive motor speeds
    for (Drivetrain.MotorLocation loc : Drivetrain.MotorLocation.values()) {
      drivetrain.setDriveMotorSpeed(loc, moduleStates.get(loc).speedMetersPerSecond / 2.0);
    }

    //TODO: Set steering angles

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
