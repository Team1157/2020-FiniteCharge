/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private WPI_TalonSRX shooterTalon;

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        shooterTalon = new WPI_TalonSRX(Constants.shooterMotorNumber);
    }

    public void setSpeed(double speed) {
        shooterTalon.set(speed);
    }

    public void stop() {
        shooterTalon.set(0);
    }

    @Override
    public void periodic() {
    }
}
