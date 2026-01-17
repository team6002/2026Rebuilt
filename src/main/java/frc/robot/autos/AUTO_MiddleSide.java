package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_MiddleSide extends SequentialCommandGroup {
    public AUTO_MiddleSide(Drive drive, SwerveDriveSimulation sim, Boolean mirrored) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
            ,new InstantCommand(()->drive.setAutoStartPose("swipehalfM2", mirrored))
            ,drive.followPath("swipehalfM2", mirrored)
            ,drive.followPath("shootfuelM2", mirrored)
            ,new ShootFuelSim(sim)
            // ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            // ,new WaitCommand(2)
            // // ,drive.followPath("shootfirstcycle")
            // ,new InstantCommand(()->sim.rotateAboutCenter(FieldConstants.HubPose.minus(drive.getPose().getTranslation()).getAngle().getRadians()))
            // ,new ShootFuelSim(sim)
            // ,drive.followPath("pickuplastcycle")
            // ,drive.followPath("shootlastcycle")
            // ,new ShootFuelSim(sim)
            // // ,drive.followPath("climb")
        );
    }
}