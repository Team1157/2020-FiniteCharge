/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLights;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Rotate the robot towards the goal using the vision system
 */
public class VisionAlign extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final Drivetrain drivetrain;
    private final VisionLights visionSubsystem;
    private final DoubleSupplier gyroAngle;
    private final NetworkTableEntry getAngleToTarget;
    private final NetworkTableEntry getTargetFound;

//    private double targetAngle;

    private NetworkTable visionTable;

    private double lastAngleToTarget = 1000; //Make sure to start on to always be different from the first reading
    private double lastAngle = 0;
    private List<Double> angles = new ArrayList<>();

    /**
     * Consturctor for the VisionAlign command
     *
     * @param drivetrainSubsystem The Drivetrain subsystem
     * @param visionLightsSubsystem The VisionLights subsystem
     */
    public VisionAlign(Drivetrain drivetrainSubsystem, VisionLights visionLightsSubsystem, DoubleSupplier getGyroAngle) {
        drivetrain = drivetrainSubsystem;
        visionSubsystem = visionLightsSubsystem;
        gyroAngle = getGyroAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

        getAngleToTarget = visionTable.getEntry("AngleToTarget");
        getTargetFound = visionTable.getEntry("TargetFound");
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        visionSubsystem.setLightState(true);
        drivetrain.configurePIDLoop(-0.02, 0.001, 0);
        drivetrain.initRotationPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = getAngleToTarget.getDouble(0); // Get angle
        if (angle != lastAngle && getTargetFound.getBoolean(false)) { // If angle has changed and target found, then
            angles.add(angle); // Add the new angle to the list
            lastAngle = angle;
        }
        if (angles.size() >= 10) { // Every ~1/3 second
            Collections.sort(angles); // Sort list to find median
            double angleToTarget = angles.get(angles.size() / 2); // Approximate median angle
            double targetAngle = (gyroAngle.getAsDouble() + angleToTarget) % 360; // Convert relative to gyro angle
            if (targetAngle < 0) {
                targetAngle += 360;
            }
            drivetrain.setRotationPIDSetpoint(targetAngle); //
            angles.clear();
        }

        drivetrain.updateRotationPID(gyroAngle.getAsDouble()); // Supply current gyro to PID to update ouptuts

//        if (getTargetFound.getBoolean(false)) {
//            double angleToTarget = getAngleToTarget.getDouble(1000);
//            if (angleToTarget != lastAngleToTarget) {
//                targetAngle = (gyroAngle.getAsDouble() + angleToTarget) % 360;
//                if (targetAngle < 0) {
//                    targetAngle += 360;
//                }
//                drivetrain.setRotationPIDSetpoint(targetAngle);
//            }
//        } else {
//            lastAngleToTarget = 1000;
//        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDriveMotors();
        visionSubsystem.setLightState(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
