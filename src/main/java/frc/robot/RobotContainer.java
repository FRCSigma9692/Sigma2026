    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;

    import static edu.wpi.first.units.Units.*;
    import java.util.function.BooleanSupplier;

    import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
    import com.ctre.phoenix6.swerve.SwerveRequest;
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.auto.NamedCommands;

    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj.smartdashboard.Field2d;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
    import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
    import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
    import frc.robot.commands.FeedCmd;
    import frc.robot.commands.FeederStop;
    import frc.robot.commands.IntakCmd;
    import frc.robot.commands.TransferCmd;
    import frc.robot.commands.TransferStop;
    import frc.robot.generated.TunerConstants;
    import frc.robot.subsystems.CommandSwerveDrivetrain;
    import frc.robot.subsystems.FeederSub;
    import frc.robot.subsystems.Hopper;
    import frc.robot.subsystems.Intake;
    import frc.robot.subsystems.TransferSub;
    import frc.robot.subsystems.Shooter;

    public class RobotContainer {
        public double AlignmentSpeed = 0.25;
        public double TransferRPM = 0.9;
        public double FeederRPM = 5000;
        public double rpm = 2600;
        public BooleanSupplier override = ()-> true;
        public Hopper hopper = new Hopper();
        public double output;
        public double RotPow45;
        private double speed = 0.6;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * speed; // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        //private Pushing transfer;

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);
        public CommandPS5Controller User1 = new CommandPS5Controller(0);    

        private final CommandXboxController joystick = new CommandXboxController(1);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private Intake intake = new Intake();
        public Shooter shooter = new Shooter(drivetrain);
        public FeederSub feeder = new FeederSub(shooter,drivetrain);
        private IntakCmd intakCmdOn = new IntakCmd(intake,0.6);
        private IntakCmd intakCmdOff = new IntakCmd(intake, 0);
        public TransferSub transfer = new TransferSub(shooter);
        private TransferCmd transferCmd  = new TransferCmd(transfer, 0.6, drivetrain);
        private FeedCmd feedCmd = new FeedCmd(feeder,0.6, drivetrain);
        public Timer timer = new Timer();
        //private ShooterAutoCmd shooterCmd = new ShooterAutoCmd(shooter);
        public Field2d field = new Field2d();
        //private CheckSparkFlex cs= new CheckSparkFlex();
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

            autoChooser = AutoBuilder.buildAutoChooser("AutoChooser");
            NamedCommands.registerCommand("Align",new InstantCommand(()-> drivetrain.rotOverride(drivetrain.rot)));
            NamedCommands.registerCommand("Shoot", Commands.run(()->shooter.runShooterRPM()));
            NamedCommands.registerCommand("StartIntake", intakCmdOn);
            NamedCommands.registerCommand("StopIntake", intakCmdOff);
            NamedCommands.registerCommand("FeederTransfer", new ParallelCommandGroup(feedCmd,transferCmd));
            SmartDashboard.putData("Auto Mode", autoChooser);
            SmartDashboard.putData("Field", field);
            configureBindings();
        }

        private void configureBindings() {
            // User1.circle().onTrue(
            //     Commands.race(drivetrain.pathFind(10.8,7.17,180.0, override),Commands.waitUntil(()->(!((Math.abs(User1.getLeftX()))<0.05 && Math.abs(User1.getLeftY())<0.05 && Math.abs(User1.getRightX())<0.03 && Math.abs(User1.getRightY())<0.03)))));
            // new Trigger(()->((Math.abs(User1.getLeftX()))<0.1 && Math.abs(User1.getLeftY())<0.1 && Math.abs(User1.getRightX())<0.1 && Math.abs(User1.getRightY())<0.1))
            // .onTrue(
            //     Commands.runOnce(()->CommandScheduler.getInstance().cancelAll()));
        
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-User1.getLeftY() * MaxSpeed * speed) // Drive forward with negative Y (forward)
                        .withVelocityY(-User1.getLeftX() * MaxSpeed * speed) // Drive left with negative X (left)
                        .withRotationalRate(-User1.getRightX() * MaxAngularRate * speed) // Drive counterclockwise with negative X (left)
                )
            );

            // Reset Robot Heading 
            User1.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
            
            // Break Mode
            User1.cross().whileTrue(drivetrain.applyRequest(() -> brake));
            
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
            );

            User1.pov(0).whileTrue(
                                    drivetrain.applyRequest(() -> 
                                    forwardStraight.withVelocityX(0.5).withVelocityY(0)));
            User1.pov(180)
                            .whileTrue(drivetrain.applyRequest(
                                            () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        
            // Shooter ----------------------                       
            joystick.leftTrigger(0.7).whileTrue(
                new ParallelCommandGroup(
                    Commands.run(() -> shooter.runShooterRPM(), shooter),
                    Commands.run(() -> transfer.runShooterRPM(TransferRPM)),
                    //ew TransferCmd(transfer,TransferRPM, drivetrain),  
                    Commands.run(() -> feeder.FeederNoPID(0.6)))
            );
            joystick.leftTrigger(0.7).whileFalse(
                new ParallelCommandGroup(
                new InstantCommand(()-> shooter.Stop()),
                new FeederStop(feeder),
                new TransferStop(transfer))
                );
            // -------------------------

            //Feeder -----------------
            // joystick.rightTrigger().whileTrue((new InstantCommand(()-> feeder.FeederNoPID(0.8))));
            // joystick.rightTrigger().whileFalse(new InstantCommand(()-> feeder.FeederNoPID(0)));
            // //------------------------

            // //Transfer -------------------
            // joystick.povUp().whileTrue(new TransferCmd(transfer, TransferRPM,drivetrain));
            // joystick.povDown().whileTrue(new TransferStop(transfer));
            // //----------------------------

            // Allignment ---------------
            User1.R1().whileTrue(  
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(-User1.getLeftY() * MaxSpeed * AlignmentSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-User1.getLeftX() * MaxSpeed * AlignmentSpeed) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.rot*MaxAngularRate*0.4)));
            // -----------------------------

            // Intake -----------------------
            joystick.y().onTrue(
                new ParallelCommandGroup(
                    new IntakCmd(intake, 0.7),
                    new TransferCmd(transfer,TransferRPM, drivetrain),
                    new InstantCommand(()-> feeder.FeederNoPID(-0.3))
                    // shooter.run(() -> shooter.runShooter(drivetrain.getShooterSpeed())),
                    // feeder.run(() -> feeder.FeederNoPID(0.6))
                )
            );
            
            joystick.a().onTrue(
                new ParallelCommandGroup(
                    new IntakCmd(intake, 0),
                    new TransferStop(transfer),
                    new InstantCommand(()-> feeder.FeederNoPID(0))
                    // shooter.run(() -> shooter.runShooter(drivetrain.getShooterSpeed())),
                    // feeder.run(() -> feeder.FeederNoPID(0.6))
                )
            );
            // -------------------------------
            
            // Stop All ---------------------------- 
            joystick.rightBumper().onTrue(
                new ParallelCommandGroup(
                    new IntakCmd(intake, 0),
                    new TransferStop(transfer),
                    new InstantCommand(()-> feeder.FeederNoPID(0)),
                    new InstantCommand(()-> shooter.Stop())
                    // shooter.run(() -> shooter.runShooter(drivetrain.getShooterSpeed())),
                    // feeder.run(() -> feeder.FeederNoPID(0.6))
                )
            );
            // --------------------------------------

            //User1.triangle().whileTrue(new CalcVel(shooterCalc, 5));

            //User1.circle().onTrue(drivetrain.pathFind(10.8,7.17,180.0, override));
            //User1.R1().whileTrue(new RunCommand(() -> drivetrain.ResetYaw(), drivetrain));
            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.

            // new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new FlyWheelCommand(flyWheel, 300));

            // Shooter
            //joystick.leftTrigger().whileTrue(new InstantCommand(()-> shooter.runShooterRPM(2600)));

            //joystick.leftTrigger().whileFalse(new ShooterCmd(shooter, 1600));//2700s
            //joystick.leftTrigger().whileFalse(new ShooterStop(shooter));//2700
            //joystick.a().onTrue(new ShooterStop(shooter));

            //joystick.whileTrue(new InstantCommand(()-> shooter.NoPID(joystick.getRightTriggerAxis()*0.5)));
            // joystick.a().onTrue(new ShooterStop(shooter).alongWith(new InstantCommand(()-> feeder.Stop())));
            // joystick.x().onTrue(new ShooterIncCommand(Shooter, rpm));
            // joystick.b().onTrue(new ShooterDecCommand(Shooter, rpm));

            // Feeder
            //joystick.povLeft().onTrue(new InstantCommand(()-> feeder.runFeeder(FeederRPM)));
            //joystick.povRight().onTrue(new InstantCommand(()-> feeder.runFeeder(-FeederRPM)));
            //joystick.leftBumper().onTrue(new InstantCommand(()-> feeder.Stop()));
            
            //joystick.povLeft().onTrue(new InstantCommand(()-> drivetrain.AutoWon()));
            //joystick.povRight().onTrue(new InstantCommand(()-> drivetrain.AutoLost()));
            //joystick.povDownLeft().whileTrue(new HopperCmd(hopper, 0.8));
            //joystick.x().onTrue(new ShooterIncCommand(fs, rpm));
            // SmartDashboard.putNumber("RPMWheel", rpm);

            // User1.L2().onTrue(
            //     drivetrain.applyRequest(() ->
            //             drive.withVelocityX(-User1.getLeftY() * MaxSpeed * speed) // Drive forward with negative Y (forward)
            //             .withVelocityY(-User1.getLeftX() * MaxSpeed * speed) // Drive left with negative X (left)
            //         .withRotationalRate(drivetrain.Rot45)));

            // joystick.rightTrigger().whileTrue(new FeedingCmd(feed, 0.6));
            //joystick.rightTrigger().whileTrue(new IntakCmd(intake, -0.8));
            
            // joystick.rightBumper().whileTrue(new IntakCmd(intake, 0)).ParallelCommandGroup(new TransferStop(transfer));
            //joystick.rightBumper().whileTrue(new CheckFlex(cs));
            //cs.setDefaultCommand(new RunCommand(()-> cs.runMootor(),cs));
            
            // joystick.b().whileTrue(drivetrain.applyRequest(() ->
            //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
            // ));

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

            // Reset the field-centric heading on left bumper press.
            drivetrain.registerTelemetry((state) -> {
                            field.setRobotPose(state.Pose);
                    });        

            drivetrain.registerTelemetry(logger::telemeterize);
        }
        

        public Command getAutonomousCommand() {
            //PPHolonomicDriveController.overrideRotationFeedback(()->drivetrain.rot);
            return autoChooser.getSelected();
            // Simple drive forward auton
            // final var idle = new SwerveRequest.Idle();
            // return Commands.sequence(
            //     // Reset our field centric heading to match the robot
            //     // facing away from our alliance station wall (0 deg).
            //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            //     // Then slowly drive forward (away from us) for 5 seconds.
            //     drivetrain.applyRequest(() ->
            //         drive.withVelocityX(0.5)
            //             .withVelocityY(0)
            //             .withRotationalRate(0)
            //     )
            //     .withTimeout(5.0),
            //     // Finally idle for the rest of auton
            //     drivetrain.applyRequest(() -> idle)
            // );
        }
    }
