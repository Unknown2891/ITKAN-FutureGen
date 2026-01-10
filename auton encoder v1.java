package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous

public class Auton extends LinearOpMode{
    
    private Limelight3A limelight;
    DcMotor fl, fr, bl, br;
    DcMotor intake, transfer;
    DcMotorEx outtakeR, outtakeL;
    int flTarget, frTarget, blTarget, brTarget;
    double COUNTS_PER_INCH = 2000/50;
    private IMU imu;
    Servo gate;
    private VisionPortal visionPortal;
    private AprilTagProcessor apriltag;
    double closed = 0.3;
    double open = 0.6;
    
    @Override
    public void runOpMode() {
        
        hardwareInit();

        waitForStart();

        if (opModeIsActive()) {
            drive(-50, -0.5);
            shoot(1550);
        }
    }
    
    public void hardwareInit(){
    
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        
        outtakeR = hardwareMap.get(DcMotorEx.class,"outtakeR");
        outtakeL = hardwareMap.get(DcMotorEx.class,"outtakeL");
        transfer = hardwareMap.get(DcMotor.class,"transfer");
        intake = hardwareMap.get(DcMotor.class,"intake");
        gate = hardwareMap.get(Servo.class,"gate");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        
        outtakeR.setDirection(DcMotor.Direction.REVERSE);
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        
        outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        gate.setPosition(closed);
    }
    public void drive(double inches, double power){
        
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        int flT = (int)(inches * COUNTS_PER_INCH);
        int frT = (int)(inches * COUNTS_PER_INCH);
        int blT = (int)(inches * COUNTS_PER_INCH);
        int brT = (int)(inches * COUNTS_PER_INCH);

        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        double avg = Math.abs(flT - fl.getCurrentPosition()) +
                     Math.abs(blT - bl.getCurrentPosition()) +
                     Math.abs(frT - fr.getCurrentPosition()) +
                     Math.abs(brT - br.getCurrentPosition());
                     
        while(avg > 400){
            
         avg = Math.abs(flT - fl.getCurrentPosition()) +
               Math.abs(blT - bl.getCurrentPosition()) +
               Math.abs(frT - fr.getCurrentPosition()) +
               Math.abs(brT - br.getCurrentPosition());
               
         avg = avg/4;
            
        }

    fl.setPower(0);
    fr.setPower(0);
    bl.setPower(0);
    br.setPower(0);
        
        return;
        
    }
    public void strafe(double inches, double power){
        
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        int flT = (int)(inches * COUNTS_PER_INCH);
        int frT = (int)(-inches * COUNTS_PER_INCH);
        int blT = (int)(-inches * COUNTS_PER_INCH);
        int brT = (int)(inches * COUNTS_PER_INCH);

        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);

        double avg = Math.abs(flT - fl.getCurrentPosition()) +
                     Math.abs(blT - bl.getCurrentPosition()) +
                     Math.abs(frT - fr.getCurrentPosition()) +
                     Math.abs(brT - br.getCurrentPosition());
                     
        while(avg > 400){
            
         avg = Math.abs(flT - fl.getCurrentPosition()) +
               Math.abs(blT - bl.getCurrentPosition()) +
               Math.abs(frT - fr.getCurrentPosition()) +
               Math.abs(brT - br.getCurrentPosition());
               
         avg = avg/4;
            
        }

    fl.setPower(0);
    fr.setPower(0);
    bl.setPower(0);
    br.setPower(0);
        
        return;
        
    }
    public double Heading(){
        
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Heading ", angles);
        telemetry.update();
        return angles.getYaw(AngleUnit.DEGREES);
    }
    private void shoot(int velocity){
        gate.setPosition(closed);
        ElapsedTime timer = new ElapsedTime();
       
        double x = 0;
        double angle  = 0;
        double distance = 0;
        double turn = 0;
        
        while(timer.seconds() < 10){
            
            telemetry.addData("velocity ", outtakeL.getVelocity());
            telemetry.update();
            
            turn = 0;
            x = 0;
            
            transfer.setPower(1);
            intake.setPower(1);
            
            boolean aligned = true;
            double tolerance = 1;
            
            int target = velocity;
            
            double currentVel = outtakeL.getVelocity();
            if(currentVel < target){
                outtakeR.setPower(1);
                outtakeL.setPower(1);
            }else{
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }
            if ((target - currentVel) < 20){
                gate.setPosition(open);
                sleep(2000);
                gate.setPosition(closed);
            }
        }
        transfer.setPower(0);
        outtakeR.setPower(0);
        outtakeL.setPower(0);
    }
    public void turn(int angle){
        angle = -angle;
        imu.resetYaw();
        double currentAngle = Heading();
        while (Math.abs(currentAngle - angle) > 0.5){
            currentAngle = Heading();
            double error = Math.abs(currentAngle - angle);
            double kp = 0.02;
            double kf = Math.signum(angle) * 0.00;
            double pow = kp * error + kf;
            pow = Math.min(Math.max(pow, -0.5),0.5);
            pow = Math.max(0.1,Math.abs(pow)) * Math.signum(pow);
            fr.setPower(-Math.signum(angle) * pow);
            fl.setPower(Math.signum(angle) * pow);
            br.setPower(-Math.signum(angle) * pow);
            bl.setPower(Math.signum(angle) * pow);
            
            telemetry.addData("current angle ", currentAngle);
            telemetry.addData("target angle ", angle);
            telemetry.addData("pow ", pow);
            telemetry.addData("error, ", error);
            telemetry.update();
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        double drift = Heading();
        turn((int)drift);
    }
    public void transfer(long power, long time){
        
        intake.setPower(power);
        transfer.setPower(power);
        sleep(time);
        sleep(time);
        
    }
    public double anglewrap(double radians) {
        while (radians > 180) {
            radians -= 360;
        }
        while (radians < -180) {
            radians += 360;
        }
          return radians;
    }

}
