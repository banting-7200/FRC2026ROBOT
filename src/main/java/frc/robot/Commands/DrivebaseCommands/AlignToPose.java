package frc.robot.Commands.DrivebaseCommands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElasticSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utilites.HelperFunctions;

public class AlignToPose extends Command {

    private final SwerveSubsystem drive;
    private final Supplier<Pose2d> targetPoseSupplier;

    private PIDController forwardPID = new PIDController(1.2, 0, 0.12);
    private PIDController strafePID = new PIDController(1.2, 0, 0.12);
    private PIDController thetaPID = new PIDController(0.04, 0, 0.5);

    private final double vxClamp = 0.8; // m/s
    private final double vyClamp = 0.8; // m/s
    private final double omegaClamp = 2.0; // rad/s

    private final double finishPosTolerance = 0.05; // 5 cm final pos tolerance
    private final double finishAngleTolerance = Math.toRadians(5); // within 5 degrees final

    private final double timeoutSeconds = 6.0;
    private double startTime;
    //TODO Switch to profiledPIDControllers for smoother and better aligning
    public AlignToPose(SwerveSubsystem drive, Supplier<Pose2d> targetPoseSupplier, PIDController forwardPID,
            PIDController strafePID, PIDController thetaPID) {
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        this.forwardPID = forwardPID;
        this.strafePID = strafePID;
        this.thetaPID = thetaPID;
        addRequirements(drive);

        // allow continuous wrap for heading
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        forwardPID.reset();
        strafePID.reset();
        thetaPID.reset();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        if (targetPose == null) {
            drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            return;
        }

        Pose2d error = targetPose.relativeTo(robotPose);
        double xError = error.getX(); // forward: + is in front of robot
        double yError = error.getY(); // left: + is to robot's left
        double thetaError = error.getRotation().getRadians() - Math.PI; // TODO why do I have to flip this 180 deg :(

        double vxCmd = forwardPID.calculate(xError, 0);
        double vyCmd = strafePID.calculate(yError, 0);
        double omegaCmd = thetaPID.calculate(thetaError, 0);

        

        // clamp outputs
        vxCmd = HelperFunctions.clamp(vxCmd, -vxClamp, vxClamp);
        vyCmd = HelperFunctions.clamp(vyCmd, -vyClamp, vyClamp);
        omegaCmd = HelperFunctions.clamp(omegaCmd, -omegaClamp, omegaClamp);

        // very small deadbands to avoid jitter
        if (Math.abs(vxCmd) < 0.005)
            vxCmd = 0.0;
        if (Math.abs(vyCmd) < 0.005)
            vyCmd = 0.0;
        if (Math.abs(omegaCmd) < 0.05)
            omegaCmd = 0.0;

        // send robot-relative chassis speeds
        drive.drive(new ChassisSpeeds(-vxCmd, -vyCmd, omegaCmd));
    }

    @Override
    public boolean isFinished() {

        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        if (targetPose == null) {
            System.out.println("TARGET POSE NULL");
            return true;
        }

        Pose2d error = targetPose.relativeTo(robotPose);
        double posDist = Math.hypot(error.getX(), error.getY());
        double angErr = Math.abs(error.getRotation().getRadians());
        ElasticSubsystem.putNumber("Theta Error", Math.toDegrees(angErr));
        ElasticSubsystem.putNumber("Position Error", posDist);
        

        if (posDist < finishPosTolerance && angErr < finishAngleTolerance)
            return true;

        if (Timer.getFPGATimestamp() - startTime > timeoutSeconds) {
            System.out.println("CENTERING COMMAND TIMED OUT");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // stop wheels
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

}