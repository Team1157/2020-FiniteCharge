package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives to specific coordinates relative to the defined 0,0 point in Drivetrain.odometry
 */
public class GotoPosition extends CommandBase {
    private Drivetrain drivetrain;
    private ProfiledPIDController pidController;

    double targetAngle;
    double targetDistance;
    double distanceTraveled;

    public GotoPosition(Drivetrain drivetrain, double x, double y) {
        this.drivetrain = drivetrain;
        targetAngle = (Math.atan(y/x) * 180 / Math.PI) - 90; // Get target angle, convert to degrees CCW from forward
        targetDistance = Math.sqrt(x*x + y*y);

        pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1));
    }

    @Override
    public void initialize() {
        drivetrain.setDesiredWheelAngle(Drivetrain.MotorLocation.FRONT_LEFT, targetAngle);
        drivetrain.setDesiredWheelAngle(Drivetrain.MotorLocation.FRONT_RIGHT, targetAngle);
        drivetrain.setDesiredWheelAngle(Drivetrain.MotorLocation.BACK_LEFT, targetAngle);
        drivetrain.setDesiredWheelAngle(Drivetrain.MotorLocation.BACK_RIGHT, targetAngle);

        pidController.setGoal(targetDistance);
        pidController.reset(0, 0);
    }

    @Override
    public void execute() {
        //pidController.calculate()
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
