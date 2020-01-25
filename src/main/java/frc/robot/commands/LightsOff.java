package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionLights;

public class LightsOff extends InstantCommand {
    private VisionLights visionLights;

    public LightsOff(VisionLights subsystem) {
        visionLights = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        visionLights.setLightState(false);
    }
}
