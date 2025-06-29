package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator; 
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.ObjectMapper;

public class SwerveSubsystem extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator swervePoseEstimator;
    private final Field2d field = new Field2d();
        
    
    
    // private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final AHRS gyro ;
    private static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.02, 0.02, Math.toRadians(0.5));// 還沒測試
    public SwerveModule[] mSwerveMods;
    // public Pigeon2 gyro;

    public SwerveSubsystem() {
         gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
         SmartDashboard.putData("Field", field);
         zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(), // Initial pose (can be set by auto or field alignment)
            ODOMETRY_STD_DEVS, // Odometry standard deviations
            VecBuilder.fill(0.0, 0.0, 0.0) // Vision standard deviations (overridden by OV9281Subsystem)
        );
        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        RobotConfig config = null;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        //   DCMotor kraken  = new DCMotor(12, 7.09, 233, 2.32, 656, 1);
        //   ModuleConfig asdf = new ModuleConfig(0.102, 5.32, 1,kraken, 100, 1);
        //   config = new RobotConfig( 58, 5.3, asdf);
          
        }
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(12, 0, 0.0), // Translation PID constants
                    new PIDConstants(0.14, 0, 0.0) // Rotation PID constants
            ),
            
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
      
    }

    public void addVisionMeasurement(Pose2d visionPose , double timestampSeconds , Matrix<N3 , N1> visionStdDevs){
        swervePoseEstimator.addVisionMeasurement((visionPose), timestampSeconds , visionStdDevs);
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
 
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[]   getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    } 

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // public void zeroHeadingg() {
    //     System.err.println("reset gyro...");
    //     gyro.setAngleAdjustment(0);
    //     gyro.reset();
    // }////////////////////////////////////////////////////////////////////////////////////////

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(( -gyro.getAngle()));

    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    

    public void stopModules(){
        for(SwerveModule swerveModule : mSwerveMods){
          swerveModule.stop();
        }
      }

    @Override
    public void periodic(){
    
        // SmartDashboard.putNumber("Robot hgHeading", gyro.getDisplacementX());
        field.setRobotPose(getPose());
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Robot hgHeading", gyro.getAngle());   
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // field.setRobotPose(getPose());
        

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
    
}