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
    private GenericHID primaryStick;
    private final Joystick secondaryStick = new Joystick(1);

    private final Timer matchTimer = new Timer();

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Gate gate = new Gate();
    private final VisionLights visionLights = new VisionLights();
    private final Climber climber = new Climber();

    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
    private final SendableChooser<Command> fieldRelativeChooser = new SendableChooser<>();
    private final ShuffleboardTab visionDebugTab = Shuffleboard.getTab("Vision Debug");
    private NetworkTableEntry visionDebugChooser = visionDebugTab.add("Vision Debug", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();

    private INPUT_MODE input_mode = INPUT_MODE.XBOX;
    public enum INPUT_MODE {
        TRUE_ONE_STICK, // Only one stick for all operations
        ONE_STICK, // One stick for driving, one for the operator
        TWO_STICK, // Translation and rotation split across two sticks
        XBOX
    }

    public static NetworkTable visionTable;
    public static NetworkTableInstance networkTableInstance;

    private boolean getClimberDirection() {
        return !secondaryStick.getRawButton(Constants.reverseClimberButtonNumber);
    }

    /**
     * Returns the current forward input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    private double getForwardInput() {
        double raw_input;
        switch (input_mode) {
            case TRUE_ONE_STICK:
            case ONE_STICK:
            case TWO_STICK:
            case XBOX:
                raw_input =  -primaryStick.getRawAxis(1);
                break;
            default:
                raw_input = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        return raw_input * getSensitivity();
    }

    /**
     * Returns the current right input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1
     */
    private double getRightInput() {
        double raw_input;
        switch (input_mode) {
            case TRUE_ONE_STICK:
            case ONE_STICK:
            case TWO_STICK:
            case XBOX:
                raw_input = primaryStick.getRawAxis(0);
                break;
            default:
                raw_input = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        return raw_input * getSensitivity();
    }

    /**
     * Returns the current rotation input from a joystick or other input device, based on the current control mode
     *
     * @return The input, from -1 to 1, with positive being counterclockwise
     */
    private double getRotationInput() {
        double raw_input;
        switch (input_mode) {
            case TRUE_ONE_STICK:
            case ONE_STICK:
                raw_input = -primaryStick.getRawAxis(2);
                break;
            case TWO_STICK:
                raw_input = -secondaryStick.getX();
                break;
            case XBOX:
                raw_input = -primaryStick.getRawAxis(4);
                break;
            default:
                raw_input = 0;
        }
        if (Math.abs(raw_input) < Constants.joystickDeadZone) {raw_input = 0;}
        return raw_input * getSensitivity();
    }

    double getSensitivity() {
        double sensitivity;
        switch (input_mode) {
            case TRUE_ONE_STICK:
            case ONE_STICK:
                sensitivity = -primaryStick.getRawAxis(3);
                break;
            case TWO_STICK:
                sensitivity = -secondaryStick.getRawAxis(2);
                break;
            case XBOX:
                sensitivity = (primaryStick.getRawAxis(3) * 2 - 1);
                break;
            default:
                sensitivity = 0;
        }
        double sensitivityMin = Constants.joystickSensitivityRange[0];
        double sensitivityMax = Constants.joystickSensitivityRange[1];
        return (sensitivityMin + (sensitivity + 1) * (sensitivityMax - sensitivityMin) / 2);
    }

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");

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
        SmartDashboard.putData("Field Relative", fieldRelativeChooser);

        SmartDashboard.putData(new LeaveInitiationLine(drivetrain));

        SmartDashboard.putData("Swerve Test",
            new SwerveTest(
                drivetrain,
                this::getRightInput,
                this::getForwardInput,
                this::getRotationInput
            ));

        CalibrateEncoders calibrateEncoders = new CalibrateEncoders(drivetrain);
        SmartDashboard.putData("Zero Absolute Encoders", calibrateEncoders);

        SmartDashboard.putData("Reset Gyro", new ResetGyro(drivetrain));
        SmartDashboard.putData("Drivetrain", drivetrain);

        gate.setDefaultCommand(new CloseGate(gate));
        // Configure the button bindings
        configureButtonBindings();
    }

    public void startTimer() {
        matchTimer.reset();
        matchTimer.start();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        System.out.println("Configuring Button Bindings");

        System.out.println(input_mode);
        if (input_mode == INPUT_MODE.XBOX) {
            primaryStick = new XboxController(0);
        } else {
            primaryStick = new Joystick(0);
        }

        GenericHID operatorStick = secondaryStick;
        if (input_mode == INPUT_MODE.TRUE_ONE_STICK) {
            operatorStick = primaryStick;
        }

        if (input_mode != INPUT_MODE.TRUE_ONE_STICK) {
            JoystickButton visionAlignButton;
            if (input_mode != INPUT_MODE.TWO_STICK) {
                visionAlignButton = new JoystickButton(primaryStick, 1);
            } else {
                visionAlignButton = new JoystickButton(secondaryStick, Constants.visionAlignButtonNumber);
            }
            visionAlignButton.whileHeld(new VisionAlign(drivetrain, visionLights, false));
        }

        JoystickButton intakeForwardsButton = new JoystickButton(operatorStick, Constants.intakeForwardsButtonNumber);
        intakeForwardsButton.whileHeld(new IntakeForwards(intake));
        JoystickButton intakeBackwardsButton = new JoystickButton(operatorStick, Constants.intakeBackwardsButtonNumber);
        intakeBackwardsButton.whileHeld(new IntakeBackwards(intake));
        JoystickButton spinUpFlywheelButton = new JoystickButton(operatorStick, Constants.spinUpFlywheelButtonNumber);
        spinUpFlywheelButton.whileHeld(new SpinUpShooter(shooter, 1.0));
        JoystickButton shootButton = new JoystickButton(operatorStick, Constants.shootButtonNumber);
        shootButton.whileHeld(new OpenGate(gate));
        JoystickButton climbUpButton = new JoystickButton(operatorStick, Constants.climbButtonNumber);
        climbUpButton.whileHeld(new Climb(climber, this::getClimberDirection));

        JoystickButton resetGyroButton = new JoystickButton(primaryStick, Constants.resetGyroButtonNumber);
        resetGyroButton.whileHeld(new ResetGyro(drivetrain));
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
        if (input_mode == INPUT_MODE.XBOX) {
            double time = matchTimer.get();
            if ((time > 104.25 && time < 104.5) || (time > 104.75 && time < 105)) {
                primaryStick.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                primaryStick.setRumble(GenericHID.RumbleType.kRightRumble, 1);
            } else {
                primaryStick.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                primaryStick.setRumble(GenericHID.RumbleType.kRightRumble, 0);
            }
        }

        visionTable.getEntry("VisionDebug").setBoolean(visionDebugChooser.getBoolean(false));

        SmartDashboard.putBoolean("Pi Online", visionTable.getEntry("FramesAvailable").getBoolean(false));
    }
}
