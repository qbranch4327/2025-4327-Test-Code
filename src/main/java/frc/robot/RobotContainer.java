// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.NewShooterCommand;
import frc.robot.commands.RotationArmCommand;
import frc.robot.commands.AutonCommands.AutonArmDownCommand;
import frc.robot.commands.AutonCommands.AutonArmUpCommand;
import frc.robot.commands.AutonCommands.AutonIndexCommand;
import frc.robot.commands.AutonCommands.AutonIntakeCommand;
import frc.robot.commands.AutonCommands.AutonIntakeCommandLong;
import frc.robot.commands.AutonCommands.AutonPassOffCommand;
import frc.robot.commands.AutonCommands.AutonShooterCommand;
import frc.robot.commands.AutonCommands.AutonTiltLongCommand;
import frc.robot.commands.AutonCommands.AutonTiltShortCommand;
import frc.robot.commands.AutonCommands.AutonTimedIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;
import frc.robot.subsystems.RotationArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final Joystick joystick3 = new Joystick(0);
    private final CommandXboxController driver2 = new CommandXboxController(1);
    private final XboxController driver3 = new XboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final IntakeSubsystem iSub;
    private final WristSubsystem wSub;
    private final VisionSubsystem vSub;
    private final RotationArmSubsystem rSub;
    private final ClimberSubsystem cSub;
    private final NewShooterSubsystem nsSub;
    private final LEDSubsystem lSub;

    private void configureBindings() {
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-joystick.getRawAxis(4) * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(joystick.getRawAxis(3) * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );

      joystick.button(13).whileTrue(drivetrain.applyRequest(() -> brake));
      joystick.button(14).whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(3), -joystick.getRawAxis(4)))
      ));

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      joystick.button(1).and(joystick.button(12)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      joystick.button(1).and(joystick.button(11)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      joystick.button(3).and(joystick.button(12)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      joystick.button(3).and(joystick.button(11)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

      // reset the field-centric heading on left bumper press
      joystick.button(13).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      drivetrain.registerTelemetry(logger::telemeterize);
  } 

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

    this.iSub = new IntakeSubsystem();
    this.wSub = new WristSubsystem();
    this.vSub = new VisionSubsystem();
    this.rSub = new RotationArmSubsystem();
    this.cSub = new ClimberSubsystem();
    // this.sSub = new ShooterSubsystem();
    this.nsSub = new NewShooterSubsystem();
    this.lSub = new LEDSubsystem();

    NamedCommands.registerCommand("AutonArmDownCommand", new AutonArmDownCommand(rSub, wSub));
    NamedCommands.registerCommand("AutonArmUpCommand", new AutonArmUpCommand(rSub, wSub));
    NamedCommands.registerCommand("AutonIntakeCommand", new AutonIntakeCommand(iSub));
    NamedCommands.registerCommand("AutonIntakeCommandLong", new AutonIntakeCommandLong(iSub));
    NamedCommands.registerCommand("AutonTimedIntakeCommand", new AutonTimedIntakeCommand(iSub));
    NamedCommands.registerCommand("AutonShooterCommand", new AutonShooterCommand(nsSub));
    // NamedCommands.registerCommand("AutonServoCommand", new AutonServoCommand(sSub));
    NamedCommands.registerCommand("AutonIntakeOn", AutonIntakeOn());
    NamedCommands.registerCommand("AutonIntakeOff", AutonIntakeOff());
    NamedCommands.registerCommand("AutonShoot", AutonShoot());
    NamedCommands.registerCommand("AutonShootAmp", AutonShootAmp());
    NamedCommands.registerCommand("AutonShootOff", AutonShootOff());
    NamedCommands.registerCommand("AutonIndexOff", AutonIndexOff());
    NamedCommands.registerCommand("AutonPassOff", new AutonPassOffCommand(iSub, nsSub, wSub, rSub));
    NamedCommands.registerCommand("AutonIndex", new AutonIndexCommand(nsSub));
    NamedCommands.registerCommand("AutonIndexMax", AutonIndexMax());
    NamedCommands.registerCommand("AutonTiltLongCommand", new AutonTiltLongCommand(nsSub));
    NamedCommands.registerCommand("AutonTiltShortCommand", new AutonTiltShortCommand(nsSub));

    autoChooser = AutoBuilder.buildAutoChooser("Auto Calibration 2m");
    SmartDashboard.putData("Auto Mode", autoChooser);

    iSub.setDefaultCommand(new IntakeCommand(iSub, driver3, joystick3));
    // iSub.setDefaultCommand(new IntakeCommand(iSub, driver2, joystick));
    // vSub.setDefaultCommand(new VisionCommand);
    rSub.setDefaultCommand(new RotationArmCommand(rSub, wSub, driver3));
    cSub.setDefaultCommand(new ClimberCommand(cSub, driver3));
    // sSub.setDefaultCommand(new ShooterCommand(sSub, driver2));
    nsSub.setDefaultCommand(new NewShooterCommand(nsSub, driver3, joystick3));
    lSub.setDefaultCommand(new LEDCommand(lSub, driver3, nsSub, iSub));
    // drivetrain.setDefaultCommand(
    //         // Drivetrain will execute this command periodically
    //         drivetrain.applyRequest(() ->
    //             drive.withVelocityX(-joystick.getRawAxis(4) * MaxSpeed) // Drive forward with negative Y (forward)
    //                 .withVelocityY(-joystick.getRawAxis(3) * MaxSpeed) // Drive left with negative X (left)
    //                 .withRotationalRate(-joystick.getRawAxis(0) * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //         )
    //     );

    //     joystick.button(13).whileTrue(drivetrain.applyRequest(() -> brake));
    //     joystick.button(14).whileTrue(drivetrain.applyRequest(() ->
    //         point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(3), -joystick.getRawAxis(4)))
    //     ));

      configureBindings();
    }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public Command AutonIntakeOn()
  {
    return new InstantCommand(()->{iSub.intakeOn(true);});
  }

  public Command AutonIntakeOff()
  {
    return new InstantCommand(()->{iSub.intakeOff();});
  }

  public Command AutonShoot()
  {
    return new InstantCommand(()->{nsSub.newshooterOn();});
  }

  public Command AutonShootAmp()
  {
    return new InstantCommand(()->{nsSub.newshooterOn();});
  }

  public Command AutonShootOff()
  {
    return new InstantCommand(()->{nsSub.newstopshooter();});
  }

  public Command AutonIndexMax()
  {
    return new InstantCommand(()->{nsSub.indexMaxAuton(true);});
  }

  public Command AutonIndexOff()
  {
    return new InstantCommand(()->{nsSub.indexOff();});
  }
}
