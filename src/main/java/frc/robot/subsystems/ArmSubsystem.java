package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.TwoJointedArmFeedforward;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    Mechanism2d mechanism = new Mechanism2d(2, 2);
    MechanismRoot2d root = mechanism.getRoot("Arm", 1, 1);
    MechanismLigament2d arm1;
    MechanismLigament2d arm2;
    double arm1Rotation = 0;
    double arm2Rotation = 0;

    Translation2d endEffector = new Translation2d();

    ArmState armState = ArmState.AWAITING_PIECE;

    TwoJointedArmFeedforward feedforward;

    public ArmSubsystem() {
        arm1 = root.append(
                new MechanismLigament2d("Arm 1", ArmConstants.arm1Length, ArmConstants.arm1StartingAngle.getDegrees()));
        arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", ArmConstants.arm2Length, ArmConstants.arm2StartingAngle.getDegrees()));

        arm1.setLineWeight(5);
        arm2.setLineWeight(5);

        endEffector = forwardKinematics(
                ArmConstants.arm1Length,
                ArmConstants.arm1StartingAngle,
                ArmConstants.arm2Length,
                ArmConstants.arm2StartingAngle);

        SmartDashboard.putData("Arm Mechanism", mechanism);

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
    }

    private void inverseKinematics(Translation2d endEffector) {
        arm2.setAngle(Units.radiansToDegrees(-Math.acos((Math.pow(endEffector.getX(), 2)
                        + Math.pow(endEffector.getY(), 2)
                        - Math.pow(arm1.getLength(), 2)
                        - Math.pow(arm2.getLength(), 2))
                / (2 * arm1.getLength() * arm2.getLength()))));

        arm1.setAngle(Units.radiansToDegrees(Math.atan2(endEffector.getY(), endEffector.getX())
                - Math.atan2(
                        arm2.getLength() * Math.sin(Units.degreesToRadians(arm2.getAngle())),
                        arm1.getLength() + arm2.getLength() * Math.cos(Units.degreesToRadians(arm2.getAngle())))));
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
        endEffector = armState.getEndEffector();
        inverseKinematics(endEffector);
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
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        Matrix<N2, N1> angles = VecBuilder.fill(Math.toRadians(arm1.getAngle()), Math.toRadians(arm2.getAngle()));
        Matrix<N2, N1> speeds = VecBuilder.fill(arm1Rotation, arm2Rotation);
        Matrix<N2, N1> acceleration = feedforward
                .calculateArmInertiaMatrix(angles)
                .inv()
                .times(feedforward
                        .calculateCoriolisMatrix(speeds, angles)
                        .times(speeds)
                        .plus(feedforward.calculateGravityMatrix(angles)))
                .times(-1);
        angles = angles.plus(speeds.times(0.02)).plus(acceleration.times(.5 * 0.02 * 0.02));
        speeds = speeds.plus(acceleration.times(0.02)).times(1);

        arm1.setAngle(Math.toDegrees(angles.get(0, 0)));
        arm2.setAngle(Math.toDegrees(angles.get(1, 0)));

        arm1Rotation = speeds.get(0, 0);
        arm2Rotation = speeds.get(1, 0);
    }

    private enum ArmState {
        AWAITING_PIECE(0.20, 0.07),
        AWAITING_DEPLOYMENT(0.22, 0.27),
        HYBRID(0.5, 0.05),
        MID(0.7, 0.6),
        HIGH(1.0, 1.1);

        private Translation2d endEffector;

        private ArmState(double x, double y) {
            endEffector = new Translation2d(x, y);
        }

        public Translation2d getEndEffector() {
            return endEffector;
        }
    }
}
