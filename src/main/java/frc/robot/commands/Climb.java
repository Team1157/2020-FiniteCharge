package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
    private Climber climber;
    private boolean direction;


    public Climb(Climber climber, boolean direction) {
        this.climber = climber;
        this.direction = direction;
    }

    @Override
    public void execute() {
        if (direction) {
            climber.up();
        }
        else {
            climber.down();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
