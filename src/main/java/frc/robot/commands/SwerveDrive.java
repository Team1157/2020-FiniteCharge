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

    private final Drivetrain drivetrain;

    private DoubleSupplier rightInput;
    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;

    /**
     * Creates a new SwerveDrive.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveDrive(Drivetrain subsystem, DoubleSupplier getRight, DoubleSupplier getForward, DoubleSupplier getRotation) {
        drivetrain = subsystem;
        rightInput = getRight;
        forwardInput = getForward;
        rotationInput = getRotation;

        // Declare dependency on the drivetrain subsystem
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double y = rightInput.getAsDouble();
        double x = forwardInput.getAsDouble();
        double z = rotationInput.getAsDouble();
        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(z) < 0.1) {
            drivetrain.stopDriveMotors();
        }

        // Create a WPILib object representing the desired velocity of the robot
        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * 2.0,
                y * 2.0,
                -z * Math.PI * 2,
                drivetrain.getGyroRotation()
        );

        drivetrain.setChassisSpeeds(desiredSpeeds);
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
