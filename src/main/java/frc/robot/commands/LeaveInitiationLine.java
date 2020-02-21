package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class LeaveInitiationLine extends CommandBase {
    private final float SECONDS_TO_DRIVE = 3;
    private Drivetrain drivetrain;
    private Timer timer = new Timer();

    public LeaveInitiationLine(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        drivetrain.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                1,
                0,
                0,
                drivetrain.getGyroRotation()
        ));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.stopDriveMotors();
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= SECONDS_TO_DRIVE);
    }
}
