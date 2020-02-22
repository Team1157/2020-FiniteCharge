package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/**
 * Align with vision, shoots the three preloaded balls, and leave the initiation line
 */
public class NoVisionAuto extends SequentialCommandGroup {
    private Intake intake;
    private Shooter shooter;
    private Gate gate;

    public NoVisionAuto(Drivetrain drivetrain, Shooter shooter, Gate gate, Intake intake, VisionLights visionLights) {
        this.intake = intake;
        this.shooter = shooter;
        this.gate = gate;

        double startingYPosition = (SmartDashboard.getNumber("Y Distance to Bumper", 77.42) + 18.25) / 39.37; //in to m
        Translation2d startingPosition = new Translation2d(13.17, startingYPosition);
        Translation2d translationToGoal = Constants.powerPortLocation.minus(startingPosition);
        double degreesToGoal = (Math.atan2(translationToGoal.getY(), translationToGoal.getX()) + 180) % 360;

        addCommands(
                new ScheduleCommand(new RotateToAngle(drivetrain, degreesToGoal)),
                new ScheduleCommand(new SpinUpShooter(shooter, 0.4)),
                new WaitCommand(2),
                new ScheduleCommand(new OpenGate(gate)),
                new WaitCommand(2.5),
                new ScheduleCommand(new IntakeForwards(intake)),
                new WaitCommand(3),
                new ScheduleCommand(new SpinUpShooter(shooter, 0)), //Stop Shooter
                new ScheduleCommand(new CloseGate(gate)),
                new LeaveInitiationLine(drivetrain)
        );
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        gate.closeGate();
    }
}
