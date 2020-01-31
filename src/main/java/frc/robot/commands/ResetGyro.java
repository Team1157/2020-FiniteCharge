package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetGyro extends InstantCommand {
    private ADXRS450_Gyro gyro;

    public ResetGyro(ADXRS450_Gyro gyro_param) {
        gyro = gyro_param;
    }

    @Override
    public void initialize() {
        gyro.reset();
    }
}
