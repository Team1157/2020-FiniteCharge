/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gate extends SubsystemBase {
    private Servo servo;

    /**
     * Creates a new Gate.
     */
    public Gate() {
        servo = new Servo(Constants.gateServoPort);
        closeGate();
    }

    public void openGate() {
        servo.set(0.24);
    }

    public void closeGate() {
        servo.set(0.686);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
