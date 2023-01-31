package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.Conversions;
import frc.lib.math.MathUtils;
import frc.lib.math.TwoJointedArmFeedforward;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import java.util.function.Function;

public class ArmSubsystem extends SubsystemBase {
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;

    private MechanismLigament2d ghostArm1;
    private MechanismLigament2d ghostArm2;

    private double joint1DesiredMotorPosition = 0;
    private double joint2DesiredMotorPosition = 0;

    private double arm1Angle = 0;
    private double arm2Angle = 0;
    private double arm1Speed = 0;
    private double arm2Speed = 0;

    LoggedReceiver desiredNetworkTablesArmPosition;

    private WPI_TalonFX joint1Motor;
    private WPI_TalonFX joint2Motor;

    private ProfiledPIDController motor1Controller;
    private ProfiledPIDController motor2Controller;

    private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(12, 6);
    private static final TrapezoidProfile.Constraints motor2Constraints = new TrapezoidProfile.Constraints(12, 6);

    Translation2d endEffector = new Translation2d();

    ArmState armState = ArmState.NETWORK_TABLES_AIM;

    TwoJointedArmFeedforward simFeedforward;

    TwoJointedArmFeedforward feedforward;

    private Timer myFunTestTimer = new Timer();

    int currentSpot = 0;

    public ArmSubsystem() {
        myFunTestTimer.start();

        arm1 = root.append(
                new MechanismLigament2d("Arm 1", ArmConstants.arm1Length, ArmConstants.arm1StartingAngle.getDegrees()));
        arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", ArmConstants.arm2Length, ArmConstants.arm2StartingAngle.getDegrees()));

        ghostArm1 = root.append(new MechanismLigament2d(
                "Ghost Arm 1", ArmConstants.arm1Length, ArmConstants.arm1StartingAngle.getDegrees()));
        ghostArm2 = ghostArm1.append(new MechanismLigament2d(
                "Ghost Arm 2", ArmConstants.arm2Length, ArmConstants.arm2StartingAngle.getDegrees()));

        ghostArm1.setLineWeight(5);
        ghostArm1.setColor(new Color8Bit(Color.kGray));
        ghostArm2.setLineWeight(5);
        ghostArm2.setColor(new Color8Bit(Color.kGray));

        arm1.setLineWeight(5);
        arm2.setLineWeight(5);

        arm1Angle = ArmConstants.arm1StartingAngle.getRadians();
        arm2Angle = ArmConstants.arm2StartingAngle.getRadians();

        joint1Motor = new WPI_TalonFX(14555);
        joint2Motor = new WPI_TalonFX(14556);

        joint1Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(ArmConstants.arm1StartingAngle.getRadians(), ArmConstants.arm1GearRatio));
        joint2Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(ArmConstants.arm2StartingAngle.getRadians(), ArmConstants.arm2GearRatio));

        endEffector = forwardKinematics(
                ArmConstants.arm1Length,
                ArmConstants.arm1StartingAngle,
                ArmConstants.arm2Length,
                ArmConstants.arm2StartingAngle);

        SmartDashboard.putData("Arm Mechanism", mechanism);

        simFeedforward = new TwoJointedArmFeedforward(
                ArmConstants.arm1Length,
                ArmConstants.arm2Length,
                ArmConstants.arm1CenterOfMass,
                ArmConstants.arm2CenterOfMass,
                ArmConstants.arm1Mass,
                ArmConstants.arm2Mass,
                ArmConstants.arm1MomentOfInertia,
                ArmConstants.arm2MomentOfInertia,
                ArmConstants.arm1GearRatio,
                ArmConstants.arm2GearRatio,
                1,
                1,
                ArmConstants.stallTorque,
                ArmConstants.stallCurrent,
                ArmConstants.freeSpeed,
                9.81);

        // for testing
        feedforward = new TwoJointedArmFeedforward(
                ArmConstants.arm1Length,
                ArmConstants.arm2Length,
                ArmConstants.arm1CenterOfMass,
                ArmConstants.arm2CenterOfMass,
                ArmConstants.arm1Mass,
                ArmConstants.arm2Mass,
                ArmConstants.arm1MomentOfInertia,
                ArmConstants.arm2MomentOfInertia,
                ArmConstants.arm1GearRatio,
                ArmConstants.arm2GearRatio,
                1,
                1,
                ArmConstants.stallTorque,
                ArmConstants.stallCurrent,
                ArmConstants.freeSpeed,
                9.81);

        motor1Controller = new ProfiledPIDController(10, 0, 0, motor1Constraints);
        motor2Controller = new ProfiledPIDController(10, 0, 0, motor2Constraints);

        motor1Controller.reset(arm1Angle);
        motor2Controller.reset(arm2Angle);

        motor1Controller.enableContinuousInput(-Math.PI, Math.PI);
        motor2Controller.enableContinuousInput(-Math.PI, Math.PI);

        desiredNetworkTablesArmPosition = Logger.tunable("/ArmSubsystem/ArmPose", new double[] {1, 0});

        setState(armState);
    }

    private Matrix<N2, N1> inverseKinematics(Translation2d endEffector) {
        try
        {    
            if (endEffector.getNorm() >= arm1.getLength() + arm2.getLength()) throw new Exception("Arm Not Long Enough");
            double arm2InverseAngle = -Math.acos((Math.pow(endEffector.getX(), 2)
                            + Math.pow(endEffector.getY(), 2)
                            - Math.pow(arm1.getLength(), 2)
                            - Math.pow(arm2.getLength(), 2))
                    / (2 * arm1.getLength() * arm2.getLength()));
            double arm1InverseAngle = Math.atan2(endEffector.getY(), endEffector.getX())
                    - Math.atan2(
                            arm2.getLength() * Math.sin(arm2InverseAngle),
                            arm1.getLength() + arm2.getLength() * Math.cos(arm2InverseAngle));
            return VecBuilder.fill(arm1InverseAngle, arm2InverseAngle);
        } catch (Exception e) {
            return VecBuilder.fill(endEffector.getAngle().getRadians(), 0);
        }
    }

    private Translation2d forwardKinematics(
            double arm1Length, Rotation2d arm1Angle, double arm2Length, Rotation2d arm2Angle) {
        return new Translation2d(
                arm1Length * arm1Angle.getCos()
                        + arm2Length * arm2Angle.plus(arm1Angle).getCos(),
                arm1Length * arm1Angle.getSin()
                        + arm2Length * arm2Angle.plus(arm1Angle).getSin());
    }

    public void setState(ArmState state) {
        armState = state;
        if (armState.getType() instanceof Brake) {
            setBrakes(true);
        } else {
            setBrakes(false);
        }
    }

    private void setBrakes(boolean brakes) {
        //TODO: set braking
    }

    public void updateArmDesiredPosition() {
        if (armState.getType() instanceof Static)
            endEffector = ((Static) armState.getType()).getEndEffector();
        else if (armState.getType() instanceof Dynamic)
            endEffector = ((Dynamic) armState.getType()).getEndEffector(this);
        Matrix<N2, N1> armAngles = inverseKinematics(endEffector);
        joint1DesiredMotorPosition = armAngles.get(0, 0);
        joint2DesiredMotorPosition = armAngles.get(1, 0);
        ghostArm1.setAngle(Math.toDegrees(joint1DesiredMotorPosition));
        ghostArm2.setAngle(Math.toDegrees(joint2DesiredMotorPosition));
    }

    public ArmState getState() {
        return armState;
    }

    public void setAwaitingPiece() {
        setState(ArmState.AWAITING_PIECE);
    }

    public void setAwaitingDeployment() {
        setState(ArmState.AWAITING_DEPLOYMENT);
    }

    public void setHybrid() {
        setState(ArmState.HYBRID);
    }

    public void setMid() {
        setState(ArmState.MID);
    }

    public void setHigh() {
        setState(ArmState.HIGH);
    }

    @Override
    public void periodic() {
        if (armState.getType() instanceof Dynamic) {
            updateArmDesiredPosition();
            executePIDFeedforward();
        } else if (armState.getType() instanceof Static) {
            executePIDFeedforward();
        }

        if (Robot.isReal()) {
            arm1Angle =
                    Conversions.falconToRadians(joint1Motor.getSelectedSensorPosition(), ArmConstants.arm1GearRatio);
            arm2Angle =
                    Conversions.falconToRadians(joint2Motor.getSelectedSensorPosition(), ArmConstants.arm2GearRatio);

            arm1Speed = Conversions.falconToRadPS(joint1Motor.getSelectedSensorVelocity(), ArmConstants.arm1GearRatio);
            arm2Speed = Conversions.falconToRadPS(joint2Motor.getSelectedSensorVelocity(), ArmConstants.arm2GearRatio);
        }

        arm1.setAngle(Math.toDegrees(arm1Angle));
        arm2.setAngle(Math.toDegrees(arm2Angle));

        executePIDFeedforward();

        /*if (myFunTestTimer.advanceIfElapsed(3.5)) {
            setState(ArmState.values()[currentSpot]);
            currentSpot++;
            currentSpot %= ArmState.values().length;
        }*/
    }

    private void executePIDFeedforward() {
        motor1Controller.setGoal(joint1DesiredMotorPosition);
        motor2Controller.setGoal(joint2DesiredMotorPosition);

        double arm1VoltageCorrection = motor1Controller.calculate(arm1Angle);
        double arm2VoltageCorrection = motor2Controller.calculate(arm2Angle);

        double arm1DesiredSpeed = motor1Controller.getSetpoint().velocity + arm1VoltageCorrection;
        double arm2DesiredSpeed = motor2Controller.getSetpoint().velocity + arm2VoltageCorrection;

        double[] voltages = feedforward.calculateFeedforwardVoltages(
                arm1Angle,
                arm2Angle,
                arm1DesiredSpeed,
                arm2DesiredSpeed,
                (arm1DesiredSpeed - arm1Speed) / 0.02,
                (arm2DesiredSpeed - arm2Speed) / 0.02);

        joint1Motor.set(MathUtils.ensureRange(voltages[0] / 12, -1, 1));
        joint2Motor.set(MathUtils.ensureRange(voltages[1] / 12, -1, 1));
    }

    @Override
    public void simulationPeriodic() {
        Matrix<N2, N1> angles = VecBuilder.fill(arm1Angle, arm2Angle);
        Matrix<N2, N1> speeds = VecBuilder.fill(arm1Speed, arm2Speed);
        Matrix<N2, N1> voltages = VecBuilder.fill(joint1Motor.get() * 12, joint2Motor.get() * 12);
        Matrix<N2, N1> acceleration = simFeedforward
                .calculateArmInertiaMatrix(angles)
                .inv()
                .times((simFeedforward.calculateMotorTorqueMatrix().times(voltages))
                        .minus(simFeedforward
                                .calculateCoriolisMatrix(speeds, angles)
                                .times(speeds))
                        .minus(simFeedforward.calculateGravityMatrix(angles))
                        .minus(simFeedforward.calculateBackEmfMatrix().times(speeds)));
        angles = angles.plus(speeds.times(0.02)).plus(acceleration.times(.5 * 0.02 * 0.02));
        speeds = speeds.plus(acceleration.times(0.02)).times(1);

        arm1Angle = angles.get(0, 0);
        arm2Angle = angles.get(1, 0);

        arm1Speed = speeds.get(0, 0);
        arm2Speed = speeds.get(1, 0);
    }

    public Translation2d getDynamicArmPosition() {
        return new Translation2d(0.5, 0).rotateBy(new Rotation2d(Timer.getFPGATimestamp()));
    }

    public Translation2d getNetworkTablesArmPosition() {
        double[] armPosition = desiredNetworkTablesArmPosition.getDoubleArray();
        try {
            return new Translation2d(armPosition[0], armPosition[1]);
        } catch (Exception e) {
            return new Translation2d(0, 0);
        }
    }

    public enum ArmState {
        AWAITING_PIECE(new Static(0.20, 0.07)),
        AWAITING_DEPLOYMENT(new Static(0.22, 0.27)),
        // HYBRID(0.5, 0.05),
        // MID(0.7, 0.6),
        // HIGH(1.0, 1.1),
        HYBRID(new Dynamic(sus -> sus.getDynamicArmPosition())),
        MID(new Dynamic(a -> a.getDynamicArmPosition())),
        HIGH(new Dynamic(a -> a.getDynamicArmPosition())),
        DANCE(new Dynamic(a -> new Translation2d(Math.cos(Timer.getFPGATimestamp()), .2))),
        NETWORK_TABLES_AIM(new Dynamic(a -> a.getNetworkTablesArmPosition())),
        BRAKE(new Brake());

        private Object type;

        public Object getType() {
            return type;
        }

        private ArmState(Object type) {
            this.type = type;
        }
    }
    
    private static class Brake {

    }

    private static class Static {
        private Translation2d endEffector;

        public Static(double x, double y) {
            endEffector = new Translation2d(x, y);
        }

        public Translation2d getEndEffector() {
            return endEffector;
        }
    }

    private static class Dynamic {
        private Function<ArmSubsystem, Translation2d> endEffectorSupplier;

        public Dynamic(Function<ArmSubsystem, Translation2d> endEffectorSupplier) {
            this.endEffectorSupplier = endEffectorSupplier;
        }

        public Translation2d getEndEffector(ArmSubsystem armSubsystem) {
            return endEffectorSupplier.apply(armSubsystem);
        }
    }
}
