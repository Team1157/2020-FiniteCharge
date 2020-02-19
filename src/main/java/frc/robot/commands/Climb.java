package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private Climber climber;

    public Climb(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void execute() {
        climber.up();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
