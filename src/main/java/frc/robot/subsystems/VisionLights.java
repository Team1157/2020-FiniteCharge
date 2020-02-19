/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionLights extends SubsystemBase {
    private Relay relay = new Relay(Constants.visionLightsRelayPort);

    /**
     * Creates a new VisionLights object.
     */
    public VisionLights() {
        setLightState(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData("Vision Lights", relay);
    }

    /**
     * Set the state of the lights, on or off
     *
     * @param state The desired state, where true is on and false is off
     */
    public void setLightState(boolean state) {
        if(state) {
            relay.set(Relay.Value.kReverse);
        } else {
            relay.set(Relay.Value.kOff);
        }
    }
}