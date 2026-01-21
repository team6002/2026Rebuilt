package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final DCMotorSim shooterSim;

    private final PIDController shooterPIDController =
            new PIDController(ShooterConstants.kPSim, ShooterConstants.kISim, ShooterConstants.kDSim);
    private final SimpleMotorFeedforward shooterFeedforward =
            new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV,
                    ShooterConstants.kA);
    private double reference = 0;
    public static double objectsInHopper = 0;

    public ShooterIOSim() {
        shooterSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), .178, ShooterConstants.kGearRatio),
                DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterCurrent = getCurrent();
        inputs.shooterVoltage = getVoltage();
        inputs.shooterReference = getReference();
        inputs.shooterVelocity = Units.radiansToDegrees(getVelocity());
    }

    @Override
    public void setReference(double reference) {
        this.reference = reference;
    }

    @Override
    public double getReference(){
        return reference;
    }

    @Override
    public void setVoltage(double voltage) {
        shooterSim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return shooterSim.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return shooterSim.getCurrentDrawAmps();
    }

    @Override
    public double getVelocity(){
        return shooterSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void PID() {
        shooterSim.setInput(shooterPIDController.calculate(
            shooterSim.getAngularVelocityRadPerSec(), reference)
            + shooterFeedforward.calculateWithVelocities(shooterSim.getAngularVelocityRadPerSec(), reference)
        );
    }

    @Override
    public void periodic() {
        shooterSim.update(0.02);
    }
}
