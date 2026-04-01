// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import java.util.function.BooleanSupplier;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Feed2Cmd;
import frc.robot.commands.FeedCmd;
import frc.robot.commands.FeedStop2;
import frc.robot.commands.FeederStop;
import frc.robot.commands.HopperCmd;
import frc.robot.commands.IntakCmd;
import frc.robot.commands.IntakStopCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.ShooterCmd2;
import frc.robot.commands.ShooterCmd3;
import frc.robot.commands.ShooterStopCmd;
import frc.robot.commands.TransferCmd;
import frc.robot.commands.TransferStop;
import frc.robot.commands.TransferStopCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feed2;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TransferSub;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

        public double AlignmentSpeed = 0.25;
        public double IntakeRobotSpeed = 1;
        public double TransferRPM = 0.9;
        public double FeederRPM = 3100;
        public double rpm = 2600;
        public BooleanSupplier override = () -> true;
        public double output;
        public double RotPow45;
        private double speed = 0.65;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity
        // private Pushing transfer;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.035).withRotationalDeadband(MaxAngularRate * 0.035) // Add a 10%
                                                                                                       // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);
        public CommandPS5Controller User1 = new CommandPS5Controller(0);

        private final CommandXboxController joystick = new CommandXboxController(1);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public Hopper hopper = new Hopper();
        public Shooter shooter = new Shooter(drivetrain);
        private Intake intake = new Intake();
        private FeederSub feeder = new FeederSub(shooter);
        // private Intake intake = new Intake()
        // public FeederSub feeder = new FeederSub(shooter, drivetrain);
        public Feed2 feeder2 = new Feed2(shooter);
        // private IntakCmd intakCmdOn = new IntakCmd(intake,0.6);
        // private IntakCmd intakCmdOff = new IntakCmd(intake, 0);
        // public TransferSub transfer = new TransferSub();
        private TransferSub transfer = new TransferSub(shooter);
        // private FeedCmd feed = new FeedCmd();
        public Timer timer = new Timer();
        // private ShooterAutoCmd shooterCmd = new ShooterAutoCmd(shooter);
        public Field2d field = new Field2d();
        // private CheckSparkFlex cs= new CheckSparkFlex();
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                // NamedCommands.registerCommand(
                // "Shooter3Sec",
                // Commands.sequence(
                // Commands.runOnce(() -> System.out.println("Shooter3Sec START")),
                // Commands.run(() -> shooter.runShooterRPMFixed7(2950), shooter)
                // .withTimeout(3.0),
                // Commands.runOnce(() -> {
                // shooter.Stop();
                // System.out.println("Shooter3Sec END");
                // }, shooter)));

                // NamedCommands.registerCommand("HopperOut", new HopperCmd(hopper, 0.3));

                // NamedCommands.registerCommand("Shooter", new ShooterCmd(shooter, 2950)); //
                // make 2950
                // //System.out.println("Shooter Registered");

                // NamedCommands.registerCommand("ShooterStop", new ShooterStopCmd(shooter));

                // NamedCommands.registerCommand("Intake", new IntakCmd(intake, -0.7));
                // NamedCommands.registerCommand("IntakeStop", new IntakStopCmd(intake));

                // NamedCommands.registerCommand("Transfer", new TransferCmd(transfer, 0.8));
                // NamedCommands.registerCommand("TransferStop", new TransferStopCmd(transfer));

                // NamedCommands.registerCommand("Feeder", new FeedCmd(feeder, 0.8));
                // NamedCommands.registerCommand("FeederStop", new FeederStop(feeder));
                // NamedCommands.registerCommand("Feeder2", new Feed2Cmd(feeder2, -0.8));
                // NamedCommands.registerCommand("Feeder2Stop", new FeedStop2(feeder2));
                // NamedCommands.registerCommand("Feeder2Stop", new Feed2Cmd(feeder2, 0));

                NamedCommands.registerCommand("Shoot_Stop",
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> shooter.Stop(), shooter),
                                                new InstantCommand(() -> feeder.FeederNoPID(0), feeder),
                                                new InstantCommand(() -> transfer.runShooterRPM(0), transfer),
                                                new InstantCommand(() -> feeder2.FeederNoPID(0), feeder2)));

                NamedCommands.registerCommand("Shoot",
                                new ParallelCommandGroup(
                                                new InstantCommand(() -> feeder.FeederNoPID(0.8), feeder),
                                                new InstantCommand(() -> transfer.runShooterRPM(0.8), transfer),
                                                new InstantCommand(() -> feeder2.FeederNoPID(-0.8), feeder2)));

                NamedCommands.registerCommand(
                                "Shooter", new InstantCommand(() -> shooter.runShooterRPMFixed7(2950), shooter));

                NamedCommands.registerCommand(
                                "ShooterStop",
                                new InstantCommand(() -> shooter.Stop(), shooter));

                NamedCommands.registerCommand(
                                "Intake", new InstantCommand(() -> intake.runIntakeForHop(-0.7), intake));

                NamedCommands.registerCommand(
                                "IntakeStop", new InstantCommand(() -> intake.runIntake(0), intake));

                // NamedCommands.registerCommand(
                // "Transfer",
                // new ParallelDeadlineGroup(
                // new WaitCommand(1),
                // new InstantCommand(() -> transfer.runShooterRPM(0.8), transfer)));

                // NamedCommands.registerCommand(
                // "TransferStop",
                // new InstantCommand(() -> transfer.stopShooter(), transfer));

                // NamedCommands.registerCommand(
                // "Feeder",
                // new ParallelDeadlineGroup(
                // new WaitCommand(1),
                // new InstantCommand(() -> feeder.FeederNoPID(0.8), feeder)));

                // NamedCommands.registerCommand(
                // "FeederStop",
                // new InstantCommand(() -> feeder.Stop(), feeder));

                // NamedCommands.registerCommand(
                // "Feeder2",
                // new ParallelDeadlineGroup(
                // new WaitCommand(1),
                // new InstantCommand(() -> feeder2.FeederNoPID(-0.8), feeder2)));

                // NamedCommands.registerCommand(
                // "Feeder2Stop",
                // new InstantCommand(() -> feeder2.FeederNoPID(0), feeder2));

                NamedCommands.registerCommand(
                                "HopperOut",
                                new HopperCmd(hopper, 0.3));

                NamedCommands.registerCommand(
                                "HopperStop",
                                new InstantCommand(() -> hopper.stop(), hopper));

                SmartDashboard.putData("Field", field);

                autoChooser = AutoBuilder.buildAutoChooser("BallLost");

                SmartDashboard.putData("Auto Mode", autoChooser);
                configureBindings();

        }

        private void configureBindings() {
                // User1.circle().onTrue(
                // Commands.race(drivetrain.pathFind(10.8,7.17,180.0,
                // override),Commands.waitUntil(()->(!((Math.abs(User1.getLeftX()))<0.05 &&
                // Math.abs(User1.getLeftY())<0.05 && Math.abs(User1.getRightX())<0.03 &&
                // Math.abs(User1.getRightY())<0.03)))));
                // new Trigger(()->((Math.abs(User1.getLeftX()))<0.1 &&
                // Math.abs(User1.getLeftY())<0.1 && Math.abs(User1.getRightX())<0.1 &&
                // Math.abs(User1.getRightY())<0.1))
                // .onTrue(
                // Commands.runOnce(()->CommandScheduler.getInstance().cancelAll()));

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-User1.getLeftY() * MaxSpeed * speed) // Drive
                                                                                                                        // (forward)
                                                .withVelocityY(-User1.getLeftX() * MaxSpeed * speed) // Drive left with
                                                                                                     // negative X
                                                                                                     // (left)
                                                .withRotationalRate(-User1.getRightX() * MaxAngularRate * speed) // Drive
                                                                                                                 // counterclockwise
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // X
                                                                                                                 // (left)
                                ));

                // Reset Robot Heading
                User1.pov(0).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // Break Mode
                User1.cross().whileTrue(drivetrain.applyRequest(() -> brake));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.leftTrigger(0.7).whileTrue(
                                new ShooterCmd(shooter, FeederRPM));
                joystick.b().whileTrue(new ShooterCmd3(shooter, 0.7));

                joystick.rightTrigger(0.5).whileTrue(new ShooterCmd2(shooter, 2950));

                joystick.axisMagnitudeGreaterThan(5, 0.1).whileTrue(
                                new RunCommand(
                                                () -> hopper.runHopper(
                                                                (MathUtil.applyDeadband(joystick.getRightY(), 0.1))
                                                                                * 0.5),
                                                hopper));
                // ----------------
                transfer.setDefaultCommand(new RunCommand(
                                () -> transfer.runShooterRPM(MathUtil.applyDeadband(-joystick.getLeftY(),
                                                0.1) * .7),
                                transfer));

                feeder.setDefaultCommand(new RunCommand(
                                () -> feeder.FeederNoPID(MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) *
                                                0.8),
                                feeder));

                feeder2.setDefaultCommand(new RunCommand(
                                () -> feeder2.FeederNoPID(MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) *
                                                -0.8),
                                feeder2));
                // --------------------
                // //Transfer -------------------
                joystick.povUp().whileTrue(
                                Commands.run(() -> shooter.runShooterRPM(), shooter));

                // Allignment ---------------
                User1.R1().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-User1.getLeftY() * MaxSpeed * AlignmentSpeed) // forward
                                                .withVelocityY(-User1.getLeftX() * MaxSpeed * AlignmentSpeed) // Drive
                                                                                                              // // X
                                                .withRotationalRate(drivetrain.rot * MaxAngularRate * speed)));

                User1.L2().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-User1.getLeftY() * MaxSpeed * AlignmentSpeed) // forward
                                                .withVelocityY(-User1.getLeftX() * MaxSpeed * AlignmentSpeed) // Drive

                                                .withRotationalRate(drivetrain.rot180 * MaxAngularRate * speed)));
                User1.R2().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-User1.getLeftY() * MaxSpeed * 0.4) // forward
                                                .withVelocityY(-User1.getLeftX() * MaxSpeed * 0.4) // Drive

                                                .withRotationalRate(-User1.getRightX() * MaxAngularRate * speed)));

                // Intake -----------------------
                joystick.leftBumper().onTrue(
                                new ParallelCommandGroup(
                                                drivetrain.applyRequest(() -> drive
                                                                .withVelocityX(-User1.getLeftY() * MaxSpeed
                                                                                * IntakeRobotSpeed * speed) // forward
                                                                .withVelocityY(-User1.getLeftX() * MaxSpeed
                                                                                * IntakeRobotSpeed * speed) // Drive
                                                                .withRotationalRate(-User1.getRightX() * MaxAngularRate
                                                                                * speed)),

                                                new IntakCmd(intake, -0.8)));

                joystick.rightBumper().onTrue(
                                new ParallelCommandGroup(
                                                new IntakCmd(intake, 0.8)));

                joystick.a().onTrue(
                                new ParallelCommandGroup(
                                                drivetrain.applyRequest(() -> drive
                                                                .withVelocityX(-User1.getLeftY() * MaxSpeed * speed) // forward
                                                                .withVelocityY(-User1.getLeftX() * MaxSpeed * speed) // Drive
                                                                .withRotationalRate(-User1.getRightX() * MaxAngularRate
                                                                                * speed)),
                                                new IntakCmd(intake, 0)
                                // new TransferStop(transfer)

                                ));

                // -------------------------------

                // Stop All ----------------------------
                joystick.x().onTrue(
                                new ParallelCommandGroup(
                                                new IntakCmd(intake, 0),
                                                new TransferStop(transfer),
                                                new InstantCommand(() -> feeder.FeederNoPID(0)),
                                                new InstantCommand(() -> shooter.Stop())
                                // shooter.run(() -> shooter.runShooter(drivetrain.getShooterSpeed())),
                                // feeder.run(() -> feeder.FeederNoPID(0.6))
                                ));
                // --------------------------------------

                // User1.pov().onTrue(new InstantCommand(() -> drivetrain.AutoWon()));
                // User1.povRight().onTrue(new InstantCommand(() -> drivetrain.AutoLost()));
                // joystick.povDownLeft().whileTrue(new HopperCmd(hopper, 0.8));
                // joystick.x().onTrue(new ShooterIncCommand(fs, rpm));
                // SmartDashboard.putNumber("RPMWheel", rpm);

                // User1.L2().onTrue(
                // drivetrain.applyRequest(() ->
                // drive.withVelocityX(-User1.getLeftY() * MaxSpeed * speed) // Drive forward
                // with negative Y (forward)
                // .withVelocityY(-User1.getLeftX() * MaxSpeed * speed) // Drive left with
                // negative X (left)
                // .withRotationalRate(drivetrain.Rot45)));

                // joystick.rightTrigger().whileTrue(new FeedingCmd(feed, 0.6));
                // joystick.rightTrigger().whileTrue(new IntakCmd(intake, -0.8));

                // joystick.rightBumper().whileTrue(new IntakCmd(intake,
                // 0)).ParallelCommandGroup(new TransferStop(transfer));
                // joystick.rightBumper().whileTrue(new CheckFlex(cs));
                // cs.setDefaultCommand(new RunCommand(()-> cs.runMootor(),cs));

                // joystick.b().whileTrue(drivetrain.applyRequest(() ->
                // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
                // -joystick.getLeftX()))
                // ));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                drivetrain.registerTelemetry((state) -> {
                        field.setRobotPose(state.Pose);
                });

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
