// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import java.util.ArrayList;
// import java.lang.reflect.Array;
// import java.util.LinkedList;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// import java.util.List;
// import java.util.LinkedList;
// import java.util.Queue;
// 
// @TeleOp(name = "SomeoneWasPlaying", group = "Vision")
// public class SomeoneWasPlaying extends LinearOpMode {
// 
//     private VisionPortal visionPortal;
//     private AprilTagProcessor aprilTag;
//     final double s_block = 0.92;
//     final double s_unblock = 1;
//      DcMotor frontLeft, frontRight, backLeft, backRight;
//     DcMotor outtakeR,outtakeL;
//     DcMotor intake;
//     Servo servo;
// 
//     @Override
//     public void runOpMode() {
//         telemetry.addLine("Ready to detect AprilTags");
//         telemetry.update();
//         config();
//         waitForStart();
//          ArrayList<Integer> q = new ArrayList<Integer><>();
//         while (opModeIsActive()) {
//           Move();
//             List<AprilTagDetection> detections = aprilTag.getDetections();
//             int targetVelocity = 1400;
//             double distance = -1;
//             for (AprilTagDetection tag : detections) {
//                  distance = tag.ftcPose.range; /// 
//             }
//              if(distance > 80){
//                      targetVelocity = 2000;
//                 }
//                 else if(distance > 60){
//                     targetVelocity = 1650;
//                 }
//                 else if(distance > 50){
//                     targetVelocity = 1600;
//                 }
//                 else if(distance > 45){
//                     targetVelocity = 1500;
//                 }
//                 else{
//                     targetVelocity = 1400;
//                 }
//                 telemetry.addData("Distance (inches)", "%.2f", distance);
//                 telemetry.update();
//              double error = targetVelocity - outtakeR.getVelocity();
//              int a = avg(q);
//              if( a > 0 && a < 50 && (gamepad1.x || gamepad2.x) ){
//                   servo.setPosition(s_unblock);
//              }
//              else{
//                   servo.setPosition(s_block);
//              }
//              outtakeR.setVelocity(targetVelocity);
//              outtakeL.setVelocity(targetVelocity);
//             //intake.setPower(0.75);
//             q.add(error);
//             if(q.size() > 5){
//                 q.poll();
//             }
//             telemetry.update();
//          
//         }
//         visionPortal.close();
//     }
//     void Move(){
//         double drive = -gamepad1.left_stick_y;  
//             double strafe = -gamepad1.left_stick_x; 
//             double turn = -gamepad1.right_stick_x;  
// 
//             double flPower = drive + strafe + turn;
//             double frPower = drive - strafe - turn;
//             double blPower = drive - strafe + turn;
//             double brPower = drive + strafe - turn;
// 
//             double max = Math.max(1.0, Math.abs(flPower));
//             max = Math.max(max, Math.abs(frPower));
//             max = Math.max(max, Math.abs(blPower));
//             max = Math.max(max, Math.abs(brPower));
// 
//             frontLeft.setPower(flPower / max);
//             frontRight.setPower(frPower / max);
//             backLeft.setPower(blPower / max);
//             backRight.setPower(brPower / max);
// 
//             if (gamepad1.left_bumper || gamepad2.left_bumper) {
//                 telemetry.addLine("left_bumper");
// 
//                 intake.setPower(-0.75); 
//             } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
//                 telemetry.addLine("right_bumper");
// 
//                 intake.setPower(0.75);
//             } else {
//                 intake.setPower(0.0); 
//             }
//     }
//     void config(){
//      
//         frontLeft = hardwareMap.get(DcMotor.class, "fl");
//         frontRight = hardwareMap.get(DcMotor.class, "fr");
//         backLeft = hardwareMap.get(DcMotor.class, "bl");
//         backRight = hardwareMap.get(DcMotor.class, "br");
//         servo = hardwareMap.get(Servo.class, "servo");
//         frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//         backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//         frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//         backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//      
//         frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    
// 
//          outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
//         outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
//         intake = hardwareMap.get(DcMotorEx.class, "intake");
//          outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         outtakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         outtakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      
//         outtakeL.setDirection(DcMotor.Direction.REVERSE);
//      
//         aprilTag = new AprilTagProcessor.Builder().build();
// 
//         visionPortal = new VisionPortal.Builder()
//                 .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                 .addProcessor(aprilTag)
//                 .build();
//     }
//  
// }
