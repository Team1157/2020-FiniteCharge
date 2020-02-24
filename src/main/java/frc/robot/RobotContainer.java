/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joystick primaryStick = new Joystick(0);
    private final Joystick secondaryStick = new Joystick(1);

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Gate gate = new Gate();
    private final VisionLights visionLights = new VisionLights();
    private final Climber climber = new Climber();

    private final SendableChooser<INPUT_MODE> inputModeChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
    private final SendableChooser<Command> fieldRelativeChooser = new SendableChooser<>();
    private final ShuffleboardTab visionDebugTab = Shuffleboard.getTab("Vision Debug");
    private NetworkTableEntry visionDebugChooser = visionDebugTab.add("Vision Debug", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    private INPUT_MODE current_input_mode = INPUT_MODE.ONE_STICK;
    public enum INPUT_MODE {
        ONE_STICK,
        TWO_STICK
    }

    public static NetworkTable visionTable;
    public static NetworkTableInstance networkTableInstance;

    /**
     * Returns the current forward input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    private double getForwardInput() {
        double raw_input;
        switch (current_input_mode) {
            case ONE_STICK:
            case TWO_STICK:
                raw_input =  -primaryStick.getY();
                break;
            default:
                raw_input = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        double sensitivity = -primaryStick.getRawAxis(3);
        double sensitivityMin = Constants.joystickSensitivityRange[0];
        double sensitivityMax = Constants.joystickSensitivityRange[1];
        return raw_input * (sensitivityMin + ((sensitivity + 1) * (sensitivityMax - sensitivityMin) / 2));
    }

    /**
     * Returns the current right input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    private double getRightInput() {
        double raw_input;
        switch (current_input_mode) {
            case ONE_STICK:
            case TWO_STICK:
                raw_input = primaryStick.getX();
                break;
            default:
                raw_input = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        double sensitivity = -primaryStick.getRawAxis(3);
        double sensitivityMin = Constants.joystickSensitivityRange[0];
        double sensitivityMax = Constants.joystickSensitivityRange[1];
        return raw_input * (sensitivityMin + (sensitivity + 1) * (sensitivityMax - sensitivityMin) / 2);
    }

    /**
     * Returns the current rotation input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1, with positive being counterclockwise
     */
    private double getRotationInput() {
        double raw_input;
        double sensitivity;
        switch (current_input_mode) {
            case ONE_STICK:
                raw_input = -primaryStick.getZ();
                sensitivity = -primaryStick.getRawAxis(3);
                break;
            case TWO_STICK:
                raw_input = -secondaryStick.getX();
                sensitivity = -secondaryStick.getRawAxis(2);
                break;
            default:
                raw_input = 0;
                sensitivity = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        double sensitivityMin = Constants.joystickSensitivityRange[0];
        double sensitivityMax = Constants.joystickSensitivityRange[1];
        return raw_input * (sensitivityMin + (sensitivity + 1) * (sensitivityMax - sensitivityMin) / 2);
    }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

        inputModeChooser.setDefaultOption("One Stick", INPUT_MODE.ONE_STICK);
        inputModeChooser.addOption("Two Stick", INPUT_MODE.TWO_STICK);
        SmartDashboard.putData("Input Mode", inputModeChooser);
        SmartDashboard.putNumber("Y Distance to Bumper", 77.42);
        SmartDashboard.putNumber("Auto Shooter Speed", 1);

        autoCommandChooser.setDefaultOption("None", null);
        autoCommandChooser.addOption("No Vision Auto", new NoVisionAuto(drivetrain, shooter, gate, intake, visionLights));
        autoCommandChooser.addOption("3 Ball Auto", new ThreeBallAuto(drivetrain, shooter, gate, intake, visionLights));
        SmartDashboard.putData("Auto Command", autoCommandChooser);

        fieldRelativeChooser.setDefaultOption("Field Relative",
            new SwerveDrive(
                drivetrain,
                this::getRightInput,
                this::getForwardInput,
                this::getRotationInput,
                true
            ));
        fieldRelativeChooser.addOption("Robot Relative",
                new SwerveDrive(
                        drivetrain,
                        this::getRightInput,
                        this::getForwardInput,
                        this::getRotationInput,
                        false
                ));
        fieldRelativeChooser.addOption("Arcade Drive", new ArcadeDrive(drivetrain, this::getForwardInput, this::getRightInput));
        SmartDashboard.putData("Field Relative Chooser", fieldRelativeChooser);

        SmartDashboard.putData(new LeaveInitiationLine(drivetrain));

        SmartDashboard.putData("Swerve Test",
                new SwerveTest(
                        drivetrain,
                        this::getRightInput,
                        this::getForwardInput,
                        this::getRotationInput
                ));

        SmartDashboard.putData("Reset Gyro", new ResetGyro(drivetrain));
        SmartDashboard.putData("Drivetrain", drivetrain);

        gate.setDefaultCommand(new CloseGate(gate));
        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton visionAlignButton;
        if (current_input_mode == INPUT_MODE.ONE_STICK) {
            visionAlignButton = new JoystickButton(primaryStick, 1);
        } else {
            visionAlignButton = new JoystickButton(secondaryStick, Constants.visionAlignButtonNumber);
        }
        visionAlignButton.whileHeld(new VisionAlign(drivetrain, visionLights, false));

        JoystickButton intakeForwardsButton = new JoystickButton(secondaryStick, Constants.intakeForwardsButtonNumber);
        intakeForwardsButton.whileHeld(new IntakeForwards(intake));
        JoystickButton intakeBackwardsButton = new JoystickButton(secondaryStick, Constants.intakeBackwardsButtonNumber);
        intakeBackwardsButton.whileHeld(new IntakeBackwards(intake));
        JoystickButton spinUpFlywheelButton = new JoystickButton(secondaryStick, Constants.spinUpFlywheelButtonNumber);
        spinUpFlywheelButton.whileHeld(new SpinUpShooter(shooter, 0.9));
        JoystickButton shootButton = new JoystickButton(secondaryStick, Constants.shootButtonNumber);
        shootButton.whileHeld(new OpenGate(gate));
        JoystickButton climbUpButton = new JoystickButton(secondaryStick, Constants.climbUpButtonNumber);
        climbUpButton.whileHeld(new Climb(climber, true)); // up
        JoystickButton climbDownButton = new JoystickButton(secondaryStick, Constants.climbDownButtonNumber);
        climbDownButton.whileHeld(new Climb(climber, false)); // down
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }

    void scheduleTeleopCommands() {
        drivetrain.setDefaultCommand(
                fieldRelativeChooser.getSelected()
        );
    }

    void periodic() {
        INPUT_MODE last_mode = current_input_mode;
        current_input_mode = inputModeChooser.getSelected();
        if (last_mode != current_input_mode) {
            System.out.println("Changing Input Mode");
            configureButtonBindings();
        }
        visionTable.getEntry("VisionDebug").setBoolean(visionDebugChooser.getBoolean(false));

        SmartDashboard.putBoolean("Pi Online", visionTable.getEntry("FramesAvailable").getBoolean(false));
    }
}
