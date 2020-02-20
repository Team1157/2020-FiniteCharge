/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public Relay winchMotor;

    /**
     * Creates a new Climber
     */
    public Climber() {
        winchMotor = new Relay(Constants.winchRelayPort);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void up() {
        winchMotor.set(Relay.Value.kForward);
    }

    public void down() {
        winchMotor.set(Relay.Value.kReverse);
    }

    public void stop() {
        winchMotor.set(Relay.Value.kOff);
    }
}
