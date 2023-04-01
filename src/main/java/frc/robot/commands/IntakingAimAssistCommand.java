package frc.robot.commands                                                                                              ;
                                                                                                                        
import edu.wpi.first.math.controller.ProfiledPIDController                                                              ;
import edu.wpi.first.math.kinematics.ChassisSpeeds                                                                      ;
import edu.wpi.first.math.trajectory.TrapezoidProfile                                                                   ;
import edu.wpi.first.wpilibj.Timer                                                                                      ;
import edu.wpi.first.wpilibj2.command.CommandBase                                                                       ;
import frc.lib.math.MathUtils                                                                                           ;
import frc.robot.Constants.VisionConstants                                                                              ;
import frc.robot.subsystems.LightsSubsystem                                                                             ;
import frc.robot.subsystems.SwerveDriveSubsystem                                                                        ;
import frc.robot.subsystems.VisionSubsystem                                                                             ;
import frc.robot.subsystems.VisionSubsystem.LimelightRawAngles                                                          ;
import java.util.function.DoubleSupplier                                                                                ;
                                                                                                                        
public class IntakingAimAssistCommand extends CommandBase                                                               {
    private VisionSubsystem visionSubsystem                                                                             ;
    private SwerveDriveSubsystem swerveDriveSubsystem                                                                   ;
                                                                                                                        
    private DoubleSupplier forward                                                                                      ;
    private DoubleSupplier strafe                                                                                       ;
    private DoubleSupplier rotate                                                                                       ;
                                                                                                                        
    private double speedModifier                                                                                        ;
                                                                                                                        
    private static final double INTAKE_FACTOR = 1                                                                       ;
                                                                                                                        
    private static final double INTAKE_DOWN_DISTANCE = 1.2                                                              ;
                                                                                                                        
    private static final double RELIABILITY_MINIMUM_ANGLE = Math.toRadians(-22)                                         ;
                                                                                                                        
    private LimelightRawAngles lastSeenLLAngles = new LimelightRawAngles(0, 0)                                          ;
                                                                                                                        
    private double maxAngularVelocity = 4                                                                               ;
                                                                                                                        
    private boolean sawCubeSoFar                                                                                        ;
                                                                                                                        
    private boolean hasCubeGottenTooClose                                                                               ;
                                                                                                                        
    private TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, 7)  ;
                                                                                                                        
    private ProfiledPIDController rotationControl = new ProfiledPIDController(6, 0, 0.4, rotationConstraints)           ;
                                                                                                                        
    private Timer timeSinceLastGoodVision = new Timer()                                                                 ;
                                                                                                                        
    public IntakingAimAssistCommand(                                                                                    
            VisionSubsystem visionSubsystem,                                                                            
            SwerveDriveSubsystem swerveDriveSubsystem,                                                                  
            LightsSubsystem lightsSubsystem,                                                                            
            DoubleSupplier forward,                                                                                     
            DoubleSupplier strafe,                                                                                      
            DoubleSupplier rotate)                                                                                      {
        this.visionSubsystem = visionSubsystem                                                                          ;
        this.swerveDriveSubsystem = swerveDriveSubsystem                                                                ;
                                                                                                                        
        addRequirements(visionSubsystem, swerveDriveSubsystem, lightsSubsystem)                                         ;
                                                                                                                        
        this.forward = forward                                                                                          ;
        this.strafe = strafe                                                                                            ;
        this.rotate = rotate                                                                                            ;
                                                                                                                        
        rotationControl.setGoal(0)                                                                                      ;
        rotationControl.setTolerance(Math.toRadians(4))                                                                 ;
        timeSinceLastGoodVision.restart()                                                                               ;}
                                                                                                                        
    @Override                                                                                                           
    public void initialize()                                                                                            {
        speedModifier = 1                                                                                               ;
        sawCubeSoFar = false                                                                                            ;
        hasCubeGottenTooClose = false                                                                                   ;}
                                                                                                                        
    @Override                                                                                                           
    public void execute()                                                                                               {
        if (visionSubsystem.hasFrontMLAngles() && visionSubsystem.isFrontLimelightAtPipeline())                         {
            LightsSubsystem.LEDSegment.MainStrip.setColor(LightsSubsystem.green)                                        ;
            LimelightRawAngles newRawAngles = visionSubsystem.getFrontMLAngles().get()                                  ;
            if (MathUtils.equalsWithinError(lastSeenLLAngles.tx(), newRawAngles.tx(), 3)                                
                    // && MathUtils.equalsWithinError(lastSeenLLAngles.ty(), newRawAngles.ty(), 3)                      
                    || timeSinceLastGoodVision.hasElapsed(0.35)                                                         
                    || !sawCubeSoFar)                                                                                   {
                lastSeenLLAngles = newRawAngles                                                                         ;
                timeSinceLastGoodVision.reset()                                                                         ;
                sawCubeSoFar = true                                                                                     ;}}
                                                                                                                        
        if (timeSinceLastGoodVision.hasElapsed(0.6) && !hasCubeGottenTooClose)                                          {
            sawCubeSoFar = false                                                                                        ;}
                                                                                                                        
        double rotationAngle = Math.toRadians(lastSeenLLAngles.tx())                                                    ;
                                                                                                                        
        if (Math.abs(lastSeenLLAngles.tx()) < 4 && timeSinceLastGoodVision.hasElapsed(0.3) && sawCubeSoFar)             {
            hasCubeGottenTooClose = true                                                                                ;}
                                                                                                                        
        if (hasCubeGottenTooClose)                                                                                      {
            rotationAngle = 0                                                                                           ;}
                                                                                                                        
        if (                                                                                                            
        /*isDriverGoingForCube() && */ sawCubeSoFar)                                                                    {
            swerveDriveSubsystem.setVelocity(                                                                           
                    new ChassisSpeeds(                                                                                  
                            -getDriverValueTowardsCube() * Math.cos(rotationAngle) * speedModifier,                     
                            -getDriverValueTowardsCube() * Math.sin(rotationAngle) * speedModifier,                     
                            MathUtils.ensureRange(                                                                      
                                    rotationControl.calculate(rotationAngle) + rotationControl.getSetpoint().velocity,  
                                    -maxAngularVelocity,                                                                
                                    maxAngularVelocity)),                                                               
                    false,                                                                                              
                    true)                                                                                               ;}
        else                                                                                                            {
            swerveDriveSubsystem.setVelocity(                                                                           
                    new ChassisSpeeds(                                                                                  
                            forward.getAsDouble() * speedModifier,                                                      
                            strafe.getAsDouble() * speedModifier,                                                       
                            rotate.getAsDouble() * speedModifier),                                                      
                    true,                                                                                               
                    true)                                                                                               ;}
        // if (shouldIntake())                                                                                          {
        //     speedModifier = INTAKE_FACTOR                                                                            ;
        //                                                                                                              }
                                                                                                                        }
                                                                                                                        
    @Override                                                                                                           
    public void end(boolean interrupted) {                                                                              }
                                                                                                                        
    private double getVelocity()                                                                                        {
        ChassisSpeeds chassisSpeeds = swerveDriveSubsystem.getVelocity()                                                ;
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)                             ;}
                                                                                                                        
    private boolean isDriverGoingForCube()                                                                              {
        // if (Math.abs(getDriverStrafeFromCube()) < 0.5) return false                                                  ;
                                                                                                                        
        if (Math.abs(Math.toRadians(lastSeenLLAngles.tx())) < Math.toRadians(30))                                       {
            return true                                                                                                 ;}
        else                                                                                                            {
            return false                                                                                                ;}}
                                                                                                                        
    private double getDriverValueTowardsCube()                                                                          {
        double cubeAngle = Math.toRadians(lastSeenLLAngles.tx())                                                        
                + swerveDriveSubsystem.getRotation().getRadians()                                                       ;
        double towardsCube = -forward.getAsDouble() * Math.cos(cubeAngle) + -strafe.getAsDouble() * Math.sin(cubeAngle) ;
        return towardsCube                                                                                              ;}
                                                                                                                        
    private double getDriverStrafeFromCube()                                                                            {
        double cubeAngle = Math.toRadians(lastSeenLLAngles.tx())                                                        
                + swerveDriveSubsystem.getRotation().getRadians()                                                       ;
        double strafeCube = forward.getAsDouble() * Math.sin(cubeAngle) + -strafe.getAsDouble() * Math.cos(cubeAngle)   ;
        return strafeCube                                                                                               ;}
                                                                                                                        
    private double getDistanceFromCube()                                                                                {
        return (VisionConstants.limelightRobotToCamera.getY() - .12 /*height of mid of cube*/)                          
                / Math.tan(Math.toRadians(lastSeenLLAngles.ty())                                                        
                        + -VisionConstants.limelightRobotToCamera.getRotation().getX() /*angle of camera*/)             ;}
                                                                                                                        
    private boolean shouldIntake()                                                                                      {
        if (!visionSubsystem.hasFrontMLAngles()) return false                                                           ;
                                                                                                                        
        if (getDistanceFromCube() / getVelocity() < (INTAKE_DOWN_DISTANCE * INTAKE_FACTOR))                             {
            return true                                                                                                 ;}
                                                                                                                        
        if (getDistanceFromCube() < 0.6) return true                                                                    ;
                                                                                                                        
        return false                                                                                                    ;}}
