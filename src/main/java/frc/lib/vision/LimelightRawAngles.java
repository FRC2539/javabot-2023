package frc.lib.vision;

public record LimelightRawAngles(double tx, double ty, double ta) {
    public LimelightRawAngles(double tx, double ty) {
        this(ty, tx, 0.0);
    }
}
