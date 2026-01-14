package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final Pose2d BlueHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(159)), new Rotation2d());
    private final Pose2d RedHubPose =
        new Pose2d(new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(159)), new Rotation2d());

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
            // IntakeIOSim.obtainFuelFromHopper();

            Pose2d robotPose = driveSim.getSimulatedDriveTrainPose();
            Pose2d hubPose = 
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? BlueHubPose : RedHubPose;

            double distance = robotPose.getTranslation().getDistance(hubPose.getTranslation());
            ShootingParams params = getShootingParams(distance);

            SimulatedArena.getInstance().addGamePieceProjectile(
                new ReefscapeAlgaeOnFly(
                    robotPose.getTranslation(),
                    new Translation2d(Units.inchesToMeters(-12), 0),
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    getTurretAngleToHub(robotPose, hubPose, params),
                    Inches.of(6),
                    MetersPerSecond.of(params.velocityMPS),
                    Degrees.of(params.angleDegrees)
                )
            );

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

    public Rotation2d getTurretAngleToHub(Pose2d robotPose, Pose2d hubPose, ShootingParams params) {
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d hubPosition = hubPose.getTranslation();

        ChassisSpeeds fieldRelativeSpeeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d robotVelocity = new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond
        );

        double projSpeed = Math.cos(Math.toRadians(params.angleDegrees)) * params.velocityMPS;

        Translation2d relativeVelocity = robotVelocity.unaryMinus();

        // Iterative solution for leading the shot
        Translation2d targetPosition = hubPosition;
        for (int i = 0; i < 3; i++) {
            Translation2d toTarget = targetPosition.minus(robotPosition);
            double timeToTarget = toTarget.getNorm() / projSpeed;

            targetPosition = hubPosition.plus(relativeVelocity.times(timeToTarget));
        }
        
        Translation2d relativeTranslation = targetPosition.minus(robotPosition);
        return relativeTranslation.getAngle();
    }
}