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

public class Intake extends SubsystemBase {
    private Relay frontRelay;
    private Relay backRelay;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        frontRelay = new Relay(Constants.frontIntakeRelayPort);
        backRelay = new Relay(Constants.backIntakeRelayPort);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        frontRelay.set(Relay.Value.kOff);
        backRelay.set(Relay.Value.kOff);
    }

    public void forward() {
        frontRelay.set(Relay.Value.kForward);
        backRelay.set(Relay.Value.kReverse);
    }

    public void backward() {
        frontRelay.set(Relay.Value.kReverse);
        backRelay.set(Relay.Value.kForward);
    }
}
