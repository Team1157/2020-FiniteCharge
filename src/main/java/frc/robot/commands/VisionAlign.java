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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionLights;

import java.util.function.DoubleSupplier;

/**
 * Rotate the robot towards the goal using the vision system
 */
public class VisionAlign extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final double PID_P_GAIN = 0; //TODO Tune controller
    private final double PID_D_GAIN = 0;
    private final double PID_I_GAIN = 0;
    private final double PID_TOLERANCE = 1;
    private final PIDController pidController;

    private final Drivetrain drivetrain;
    private final VisionLights visionSubsystem;
    private final DoubleSupplier gyro;
    private final NetworkTableEntry getAngleToTarget;
    private final NetworkTableEntry getTargetFound;

    private double targetAngle;

    private NetworkTable visionTable;

    private double lastAngleToTarget = 1000; //Make sure to start on to always be different from the first reading

    /**
     * Consturctor for the VisionAlign command
     *
     * @param drivetrainSubsystem The Drivetrain subsystem
     * @param visionLightsSubsystem The VisionLights subsystem
     */
    public VisionAlign(Drivetrain drivetrainSubsystem, VisionLights visionLightsSubsystem, DoubleSupplier getGyroAngle) {
        drivetrain = drivetrainSubsystem;
        visionSubsystem = visionLightsSubsystem;
        gyro = getGyroAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);

        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

        getAngleToTarget = visionTable.getEntry("AngleToTarget");
        getTargetFound = visionTable.getEntry("TargetFound");

        pidController = new PIDController(
                PID_P_GAIN,
                PID_I_GAIN,
                PID_D_GAIN
        );
        pidController.setTolerance(PID_TOLERANCE);
        pidController.enableContinuousInput(0, 360);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        visionSubsystem.setLightState(true);
        drivetrain.angleWheelsForRotation();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (getTargetFound.getBoolean(false)) {
            double angleToTarget = getAngleToTarget.getDouble(1000);
            if (angleToTarget != lastAngleToTarget) {
                targetAngle = (gyro.getAsDouble() + angleToTarget) % 360;
                if (targetAngle < 0) {
                    targetAngle += 360;
                }
                pidController.setSetpoint(targetAngle);
            }
        } else {
            lastAngleToTarget = 1000;
        }

        double rotationSpeed = pidController.calculate(gyro.getAsDouble());
        for (Drivetrain.MotorLocation motorLocation: Drivetrain.MotorLocation.values()) {
            if (drivetrain.areAllWheelsWithinTolerance()) {
                drivetrain.setDriveMotorSpeed(motorLocation, rotationSpeed);
            } else {
                drivetrain.setDriveMotorSpeed(motorLocation, 0);
            }
        }
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
