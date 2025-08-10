package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.OV9281Subsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.sql.XAConnection;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.w3c.dom.xpath.XPathNamespace;

public class Apriltag_cmd  extends Command{
    private final SwerveSubsystem swerveSubsystem ;
    private final OV9281Subsystem visionSubsystem;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController RotationController;
    private final double Xmeter;
    private final double Ymeter;

    private static final double MAX_ANGULAR_SPEED = 3;
    private static final double MaxSpeed = 3;

    private static final double Xok = 0.05;
    private static final double Yok = 0.05;
    private static final double RotationokOk = 2;

    public Apriltag_cmd(SwerveSubsystem swerveSubsystem , OV9281Subsystem visionSubsystem, double Xmeter , double Ymeter){
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.xController = new PIDController(0, 0, 0);
        this.yController = new PIDController(0, 0, 0);
        this.RotationController = new PIDController(0, 0, 0);
        this.Xmeter = Xmeter;
        this.Ymeter = Ymeter;
       
        addRequirements(swerveSubsystem , visionSubsystem);
    }

    @Override
    public void initialize(){
        xController.setSetpoint(Xmeter);
        yController.setSetpoint(Ymeter);
        RotationController.setSetpoint(0);
        

        xController.setTolerance(Xok);
        yController.setTolerance(Yok);
        RotationController.setTolerance(RotationokOk);
        
    }





    @Override
    public void execute(){

        Transform3d target = visionSubsystem.getBestTarget();
        

        if(target == null){
            swerveSubsystem.drive(new Translation2d( 0, 0) , 0 , false , false);
            return;
        }

        double Xerror = target.getX();
        double Yerror = target.getY();
        double RotationError = target.getRotation().getZ() *(180 / Math.PI);
        
        double Xspeed = xController.calculate(Xerror);
        double Yspeed = yController.calculate(Yerror);
        double RotationSpeed = RotationController.calculate(RotationError);

        Xspeed = Math.copySign(Math.min(Math.abs(Xspeed), MaxSpeed), Xspeed);
        Yspeed = Math.copySign(Math.min(Math.abs(Yspeed), MaxSpeed), Yspeed);
        RotationSpeed = Math.copySign(Math.min(Math.abs(RotationSpeed), MAX_ANGULAR_SPEED), RotationSpeed);

        swerveSubsystem.drive(new Translation2d(Xspeed , Yspeed) , RotationSpeed , false , false);

        SmartDashboard.putNumber("AprilTag_Target_X_Error", Xerror);
        SmartDashboard.putNumber("AprilTag_Target_Y_Error", Yerror);
        SmartDashboard.putNumber("AprilTag_Target_Yaw_Error", RotationError);




    }



    @Override
    public void end(boolean interrupted){
        swerveSubsystem.drive(new Translation2d( 0 , 0) , 0 , false , false);

    }

    @Override
    public boolean isFinished(){
      

        Transform3d target = visionSubsystem.getBestTarget();
        if (target == null) {
            return false;
        }

      

        return xController.atSetpoint() && yController.atSetpoint() && RotationController.atSetpoint();
    }




    

}
