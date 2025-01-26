package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "RobotCentricTeleop")
public class RobotCentricTeleop extends LinearOpMode {
    
    // motors
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    
    // intake
    Servo blocker = null;
    DcMotor intake = null;
    
    // slider
    DcMotor rslider = null;
    DcMotor lslider = null;

    // arm
    DcMotor arm = null;
    final double minArmPos = 0;
    final double maxArmPos = 80;
    double armSpeed = 1;
    int armTarget = 0;
    int rsliderTarget = -100;
    int lsliderTarget = 100;

    // pid
    static double kp = 0.5;
    static double ki = 0;
    static double kd = 0.001;
    double previousError = 0;
    double integralSum = 0;
    ElapsedTime time = new ElapsedTime();
    
    boolean printMessages = true;
    

    @Override
    public void runOpMode() {
        
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        blocker = hardwareMap.get(Servo.class,"blocker");
        intake = hardwareMap.get(DcMotor.class,"intake");
        arm = hardwareMap.get(DcMotor.class,"arm");
        rslider = hardwareMap.get(DcMotor.class,"rslider");
        lslider = hardwareMap.get(DcMotor.class,"lslider");
        
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);

        double drive, turn, strafe;
        double flpower, frpower, blpower, brpower;
        waitForStart();

        armTarget = -200;

        while (opModeIsActive()) {
            
            drive = -gamepad1.left_stick_y + (-gamepad2.left_stick_y);
            strafe = gamepad1.left_stick_x + gamepad2.left_stick_x;
            turn = gamepad1.right_stick_x + gamepad2.right_stick_x;
            
            frpower = drive - turn - strafe;
            brpower = drive - turn + strafe;
            flpower = drive + turn + strafe;
            blpower = drive + turn - strafe;
            
            double maxPower = Math.max(Math.abs(frpower),
                                Math.max(Math.abs(brpower),
                                Math.max(Math.abs(flpower),
                                Math.abs(blpower))));
            if(maxPower > 1){
                frpower /= maxPower;
                brpower /= maxPower;
                flpower /= maxPower;
                blpower /= maxPower;
            }
            
            // move the sliders up
            if(gamepad1.dpad_right || gamepad2.dpad_right) {
                rsliderTarget = -1700;
                lsliderTarget = 1700;
            }
            
            // move the sliders down
            if(gamepad1.dpad_left || gamepad2.dpad_left) {
                rsliderTarget = -100;
                lsliderTarget = 100;
            }
            
            // increment the sliders to the left
            if(gamepad1.left_bumper || gamepad2.left_bumper) {
                rsliderTarget = rslider.getCurrentPosition() + 30;
                lsliderTarget = lslider.getCurrentPosition() - 30;
            }
            
            // increment the sliders to the right
            if(gamepad1.right_bumper || gamepad2.right_bumper) {
                rsliderTarget = rslider.getCurrentPosition() - 30;
                lsliderTarget = lslider.getCurrentPosition() + 30;
            }
            
            // move the arm into a upright position
            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                armTarget = -2000; 
            }
            
            // move the arm to be just above the floor
            if(gamepad1.dpad_down || gamepad2.dpad_down) {
                armTarget = -300; 
            }
            
            // increase the arm angle
            if((gamepad1.right_trigger > 0.05) || (gamepad2.right_trigger > 0.05)) {
                armTarget = arm.getCurrentPosition() - 30; 
            }
            
            // decrease the arm angle
            if((gamepad1.left_trigger > 0.05) || (gamepad2.left_trigger > 0.05)) {
                armTarget = arm.getCurrentPosition() + 30;
            }
            
            // turn on intake
            if(gamepad1.a || gamepad2.a) {
                intake.setPower(-0.6);
            }
            
            // turn off intake
            if(gamepad1.b || gamepad2.b) {
                intake.setPower(0);
            }
            
            // open the blocker
            if(gamepad1.y || gamepad2.y) {
                blocker.setPosition(1);
                intake.setPower(-0.2);
            }
            
            // close the blocker
            if(gamepad1.x || gamepad2.x) {
                blocker.setPosition(0.8);
            }
            
            frontright.setPower(frpower);
            backright.setPower(brpower);
            frontleft.setPower(flpower);
            backleft.setPower(blpower);
          
            arm.setTargetPosition(armTarget);
            arm.setPower(1);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            rslider.setTargetPosition(rsliderTarget);
            rslider.setPower(1);
            rslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            lslider.setTargetPosition(lsliderTarget);
            lslider.setPower(1);
            lslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            if(printMessages) {
                telemetry.addData("Status", "Running");
                telemetry.addData("frpower", frpower);
                telemetry.addData("flpower", flpower);
                telemetry.addData("blpower", blpower);
                telemetry.addData("brpower", brpower);
                telemetry.addData("arm position", arm.getCurrentPosition());
                telemetry.addData("rslider position", rslider.getCurrentPosition());
                telemetry.addData("lslider position", lslider.getCurrentPosition());
                telemetry.update();
            }
        }

        resetRobot();
    }
    
    public void resetRobot() {
        
        rslider.setTargetPosition(-100);
        rslider.setPower(1);
        rslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
        lslider.setTargetPosition(100);
        lslider.setPower(1);
        lslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        while(rslider.isBusy() || lslider.isBusy()) {
            idle();
        }
        
        intake.setPower(0);
        
    }
}
