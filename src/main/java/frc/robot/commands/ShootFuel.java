package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;

public class ShootFuel extends Command {
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;

    // Shooting table: {distance (meters), angle (degrees), velocity (m/s)}
    private static final double[][] SHOOTING_TABLE = {
        {1.50, 81.7, 7.00},
        {2.50, 78.1, 7.62},
        {3.50, 75.4, 8.25},
        {4.50, 73.6, 8.88},
        {5.50, 72.2, 9.50}
    };

    // Shooter offsets from robot center (in robot-relative coordinates)
    private static final Translation2d CENTER_SHOOTER_OFFSET = 
        new Translation2d(Units.inchesToMeters(-12), 0);
    private static final Translation2d LEFT_SHOOTER_OFFSET = 
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(6));
    private static final Translation2d RIGHT_SHOOTER_OFFSET = 
        new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(-6));

    private final Pose2d BlueHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(159)), null);
    private final Pose2d RedHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(159)), null);

    public ShootFuel(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
    }

    @Override
    public void execute(){
        if(timer > 15 && IntakeIOSim.numObjectsInHopper() > 0){
            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
            Pose2d hubPose = 
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? BlueHubPose : RedHubPose;

            double distance = robotPose.getTranslation().getDistance(hubPose.getTranslation());
            ShootingParams params = getShootingParams(distance);

            // Shoot up to 3 balls (one from each shooter)
            int ballsToShoot = Math.min(3, IntakeIOSim.numObjectsInHopper());
            
            for (int i = 0; i < ballsToShoot; i++) {
                IntakeIOSim.obtainFuelFromHopper();
                
                // Select shooter offset based on which ball we're shooting
                Translation2d shooterOffset;
                if (i == 0) {
                    shooterOffset = CENTER_SHOOTER_OFFSET;
                } else if (i == 1) {
                    shooterOffset = LEFT_SHOOTER_OFFSET;
                } else {
                    shooterOffset = RIGHT_SHOOTER_OFFSET;
                }

                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeAlgaeOnFly(
                        robotPose.getTranslation(),
                        shooterOffset,
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        robotPose.getRotation(),
                        Inches.of(6),
                        MetersPerSecond.of(params.velocityMPS),
                        Degrees.of(params.angleDegrees)
                    )
                );
            }

            timer = 0;
        } else {
            timer++;
        }
    }

    @Override
    public boolean isFinished(){
        return IntakeIOSim.numObjectsInHopper() <= 0;
    }

    private record ShootingParams(double angleDegrees, double velocityMPS) {}

    private ShootingParams getShootingParams(double distance) {
        // Clamp to table bounds
        if (distance <= SHOOTING_TABLE[0][0]) {
            return new ShootingParams(SHOOTING_TABLE[0][1], SHOOTING_TABLE[0][2]);
        }
        if (distance >= SHOOTING_TABLE[SHOOTING_TABLE.length - 1][0]) {
            int last = SHOOTING_TABLE.length - 1;
            return new ShootingParams(SHOOTING_TABLE[last][1], SHOOTING_TABLE[last][2]);
        }
        
        // Find the two points to interpolate between
        for (int i = 0; i < SHOOTING_TABLE.length - 1; i++) {
            if (distance >= SHOOTING_TABLE[i][0] && distance <= SHOOTING_TABLE[i + 1][0]) {
                double d0 = SHOOTING_TABLE[i][0];
                double d1 = SHOOTING_TABLE[i + 1][0];
                double t = (distance - d0) / (d1 - d0);  // interpolation factor
                
                double angle = SHOOTING_TABLE[i][1] + t * (SHOOTING_TABLE[i + 1][1] - SHOOTING_TABLE[i][1]);
                double velocity = SHOOTING_TABLE[i][2] + t * (SHOOTING_TABLE[i + 1][2] - SHOOTING_TABLE[i][2]);
                
                return new ShootingParams(angle, velocity);
            }
        }
        
        // Fallback (shouldn't reach here)
        return new ShootingParams(75.0, 7.0);
    }
}