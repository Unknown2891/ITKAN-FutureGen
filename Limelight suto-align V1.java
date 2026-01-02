package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Limelight extends OpMode {
    
    //Motors and other things declaration;
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    DcMotor fr, fl, br, bl, outtakeR, outtakeL, transfer, intake;
    Servo gate;
    
    //PID stuff:
    double kp = 0.02;
    double ki = 0.001;
    double kd = 0.004;
    
    //PID stuff again;
    double target = 0;
    double error = 0;
    double previousError = 0;
    double integral = 0;
    double derivative = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    
    //align;
    boolean autoAligning = false;

    @Override
    public void init(){
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        fr = hardwareMap.get(DcMotor.class,"fr");
        fl = hardwareMap.get(DcMotor.class,"fl");
        br = hardwareMap.get(DcMotor.class,"br");
        bl = hardwareMap.get(DcMotor.class,"bl");
        
        outtakeR = hardwareMap.get(DcMotor.class,"bl");
        outtakeL = hardwareMap.get(DcMotor.class,"bl");
        transfer = hardwareMap.get(DcMotor.class,"bl");
        intake = hardwareMap.get(DcMotor.class,"bl");
        gate = hardwareMap.get(Servo.class,"gate");
        
        limelight.pipelineSwitch(0);
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void start() {
        limelight.start();
        pidTimer.reset();
    }

    @Override
    public void loop() {
    
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        
        if (gamepad1.a) {
            autoAligning = !autoAligning;
            if (autoAligning) {
                integral = 0;
                previousError = 0;
                pidTimer.reset();
            } else {
                stopMotors();
            }
        }
        
        if (autoAligning) {
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                distance = getDistaceFromTag(llResult.getTa());
                
                double tx = llResult.getTx();
                
                double turnPower = calculatePID(tx);
                
                setMotorPowers(-turnPower, turnPower, -turnPower, turnPower);
                
                telemetry.addData("Status", "AUTO-ALIGNING");
                telemetry.addData("Tx", tx);
                telemetry.addData("Turn Power", turnPower);
                telemetry.addData("Error", error);
                
                if (Math.abs(tx) < 0.5) {
                    stopMotors();
                }
            } else {
                stopMotors();
            }
        } else {
            
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;
            
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);

        }
        
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistaceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
        }
        if(gamepad1.x){
           double power = distance / 0.3;
           
           outtakeR.setPower(power);
           //outtakeL.setPower
        }
        if(gamepad1.left_bumper){
            transfer.setPower(1);
            gate.setPosition(4);
        }else{
            transfer.setPower(0);
            gate.setPosition(6);
        }
        if(gamepad1.right_bumper){
            transfer.setPower(1);
            intake.setPower(1);
        }else{
            transfer.setPower(1);
            intake.setPower(0);
        }
        
        telemetry.update();
    }
    
    private double calculatePID(double tx) {
        error = target - tx;
        
        double proportional = error * kp;
        
        if (Math.abs(error) < 10) { 
            integral += error * pidTimer.seconds();
        } else {
            integral = 0;
        }
        double integralTerm = integral * ki;
        
        derivative = (error - previousError) / pidTimer.seconds();
        double derivativeTerm = derivative * kd;
        
        double output = proportional + integralTerm + derivativeTerm;
        
        if (Math.abs(output) > 0.4) {
            output = Math.signum(output) * 0.4;
        }
        
        if (Math.abs(error) > 0.5 && Math.abs(output) < 0.1) {
            output = Math.signum(output) * 0.1;
        }
        
        previousError = error;
        pidTimer.reset();
        
        return output;
    }
    
    private void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    
    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
    
    public double getDistaceFromTag(double ta){
        double scale = 150;
        double distance = (scale / ta);
        return distance;
    }
}
