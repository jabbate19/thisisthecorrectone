package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.controllers.AxisButton;
import frc.lib.controllers.SeaKingXboxController;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.*;

public class RobotContainer {

    public final SeaKingXboxController driverController;

    public final DrivetrainSubsystem drivetrainSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final SpindexerSubsystem spindexerSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    //public final WoFSubsystem wofSubsystem;

    public RobotContainer() {
        // controllers
        driverController = new SeaKingXboxController(0);

        // Subsystems
        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        spindexerSubsystem = new SpindexerSubsystem();
        shooterSubsystem = new ShooterSubsystem();
       // wofSubsystem = new WoFSubsystem();

        configureDefaultCommands();
        configureButtonBindings();
    }
    public double GetDriverRawAxis(int axis) {
        return driverController.getRawAxis(axis);
  
     }
    
    
    private void configureDefaultCommands() {
        

        // slow spin by default
        spindexerSubsystem.setDefaultCommand(new RunCommand(
                spindexerSubsystem::slowSpin, spindexerSubsystem));

        // You can set default commands in here for subsystems to reference the controller easier.
        // This is will be active whenever another drivetrain command isnt, but I don't think you have any so you're all good
        drivetrainSubsystem.setDefaultCommand( new RunCommand( () -> drivetrainSubsystem.setDrivePowers(  () -> GetDriverRawAxis(1), () -> GetDriverRawAxis(5) ) ) );
    }

    private void configureButtonBindings() {
       


        driverController.leftBumper
                .whileHeld(() -> {
                    intakeSubsystem.startIntaking();
                    spindexerSubsystem.intakingSpin();
                }, intakeSubsystem, spindexerSubsystem)
                .whenReleased(intakeSubsystem::stopIntaking, intakeSubsystem);

        AxisButton rightTrigger = new AxisButton(driverController, 2);
        rightTrigger
                .whenPressed(() -> {
                    spindexerSubsystem.speedySpin();
                    shooterSubsystem.setRPM(4000);
                }, spindexerSubsystem)
                .whileHeld(shooterSubsystem::autoShoot)
                .whenReleased(() -> shooterSubsystem.setRPM(0));

        //driverController.aButton
                //.whenPressed(wofSubsystem::extend, wofSubsystem)
                //.whenReleased(wofSubsystem::retract);

       // driverController.bButton
               // .whenPressed(wofSubsystem::spinClockwise, wofSubsystem)
               // .whenReleased(wofSubsystem::stopSpin, wofSubsystem);
    }
}