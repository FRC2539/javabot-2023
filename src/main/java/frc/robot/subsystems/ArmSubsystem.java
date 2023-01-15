package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    Mechanism2d mechanism = new Mechanism2d(2, 2);
    MechanismRoot2d root = mechanism.getRoot("Arm", 1, 1);
    MechanismLigament2d arm1;
    MechanismLigament2d arm2;

    Translation2d endEffector = new Translation2d();

    ArmState armState = ArmState.AWAITING_PIECE;

    Timer demoTimer = new Timer();

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

        demoTimer.start();
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
                arm1Length * arm1Angle.getCos() + arm2Length * arm2Angle.plus(arm1Angle).getCos(),
                arm1Length * arm1Angle.getSin() + arm2Length * arm2Angle.plus(arm1Angle).getSin());
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
    public void periodic() {
        if(demoTimer.advanceIfElapsed(1.0)) {
            switch (armState) {
                case AWAITING_PIECE:
                    setAwaitingDeployment();
                    break;
                case AWAITING_DEPLOYMENT:
                    setHybrid();
                    break;
                case HYBRID:
                    setMid();
                    break;
                case MID:
                    setHigh();
                    break;
                case HIGH:
                    setAwaitingPiece();
                    break;
            }
        }
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
