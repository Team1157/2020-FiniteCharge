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
  public Relay relay = new Relay(Constants.visonLightsRelayPort);

  /**
   * Creates a new VisionLights object.
   */
  public VisionLights() {
    relay.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Relay", relay);
    // This method will be called once per scheduler run
  }
}