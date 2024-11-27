package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;

// This is far side Blue
@Autonomous(name="redSpecimenSide", group="Auto2024")
public class firstAutonomousProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    GoBildaPinpointDriver odo;

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        double angle = radians*180/Math.PI;
        // keep in mind that the result is in radians
        return angle;
    }
    public void turnLeft(double power){
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public void strafeRight(double power){
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void softTurnLeft(double power){
        backLeft.setPower(power/2);
        frontRight.setPower(power);
        frontLeft.setPower(power/2);
        backRight.setPower(power);
    }
    public void softTurnRight(double power){
        backRight.setPower(power/2);
        frontLeft.setPower(power);
        frontRight.setPower(power/2);
        backLeft.setPower(power);
    }


    public void turnRight(double power){
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void stopRobot(){
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
    }
    public void driveForward(double power){
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
    }
    public void driveBackwardCorrection(double angle, double power, int distance, double pos) {
        String turn = "";
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveForward(power);
//            if (angle <= -5) {
//                softTurnLeft(power);
//                turn = "left";
//            } else if (angle >= 5) {
//                softTurnRight(power);
//                turn = "right";
//            } else if (angle < 5 && angle > -5) {
//                driveForward(power);
//                turn = "forward";
//            } else {
//                stopRobot();
//            }

        telemetry.addData("turn", turn);
        telemetry.update();
    }
    public void driveForwardCorrection(double angle, double power, int distance, double pos){
        String turn = "";
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        driveForward(-power);
//            if(angle <=-5){
//                softTurnLeft(-power);
//                turn = "left";
//            }
//            else if (angle>= 5) {
//                softTurnRight(-power);
//                turn = "right";
//            }
//            else if (angle <5 && angle > -5) {
//                driveForward(-power);
//                turn = "forward";
//            }
//            else{
//                stopRobot();
//            }

        telemetry.addData("turn", turn);
        telemetry.update();


//        if(yPos > 3){
//            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//        if(yPos < 3){
//            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//         if(angle <-5){
//             backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//             frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//             frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//             backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//         } else if (angle > 5) {
//             backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//             frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//             frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//             backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//         }
//         else {
//             backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//             frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//             frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//             backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//         }

    }
    public void turnDegrees(double nAngle, double power){
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading()))); telemetry.addData("Position", data);
            double angle = (angleWrap(odo.getHeading())); odo.bulkUpdate(); turnRight(power);
            if (Math.abs(angle)>nAngle) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
    }
    public void driveDistanceForward(double power, int distance, double xPlace){
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        if (xPlace > distance){
            if (xPlace > distance/2) {
                backLeft.setPower(power);
                frontRight.setPower(power);
                frontLeft.setPower(power);
                backRight.setPower(power);
                telemetry.update();
            }
            if (xPlace < distance/2){
                backLeft.setPower(power/2);
                frontRight.setPower(power/2);
                frontLeft.setPower(power/2);
                backRight.setPower(power/2);
                telemetry.update();
            }
        }
        else {
            backLeft.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
        }
    }

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.resetPosAndIMU();


        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        runtime.reset();
        odo.setOffsets(65.0, -130); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        odo.resetPosAndIMU();
        double angle = angleWrap(odo.getHeading());
        if (opModeIsActive()) {
            odo.bulkUpdate();

////            driveDistanceForward(0.6, -500, pos.getPosition().x);
//            driveForward(0.2, angleWrap(odo.getHeading()));
            while (true) { // go forward for the initial hang
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.4, 600, pos.getPosition().x);
                if(pos.getPosition().x >= 600) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            sleep(3000);
            while (true) { // back up from the hang
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.2, 590, pos.getPosition().x);
                if(pos.getPosition().x <= 599) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            sleep(333);
            while (true) { // strafe to the blocks place
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().y >= 600) {
                    break;
                }
                strafeRight(0.4);
                if (pos.getPosition().y >= 600) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            turnDegrees(-3,-0.2);
            stopRobot();
            odo.resetPosAndIMU();
            sleep(333);
            while (true) { // go up to the 1st block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().x >= 600) {
                    break;
                }
                driveForwardCorrection(angle, 0.6, 550, pos.getPosition().x);
                if (pos.getPosition().x >= 600) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            sleep(333);
            telemetry.update();
            stopRobot();
            while (true) { // strafe over 1st block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().y >= 300) {
                    break;
                }
                strafeRight(0.4);
                if (pos.getPosition().y >= 300) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            sleep(333);
            while (true) { // push 1st block into the zone
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.6, -200, pos.getPosition().x);
                if (pos.getPosition().x <= -200) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            odo.resetPosAndIMU();
            sleep(333);
            while (true) { // go up for the second block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().x >= 800) {
                    break;
                }
                driveForwardCorrection(angle, 0.6, 950, pos.getPosition().x);
                if (pos.getPosition().x >= 800) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            sleep(500);
            while (true) { // strafe over the 2nd block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().y >= 145) {
                    break;
                }
                strafeRight(0.4);
                if (pos.getPosition().y >= 145) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            sleep(333);
            while (true) { // push the second block into the zone
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.6, 250, pos.getPosition().x);
                if (pos.getPosition().x <= 30) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            odo.resetPosAndIMU();
            stopRobot();
            sleep(500);
            telemetry.update();
            while (true) { // move up a little for the human player
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().x >= 250) {
                    break;
                }
                driveForwardCorrection(angle, 0.5, 200, pos.getPosition().x);
                if (pos.getPosition().x >= 250) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            turnDegrees(10, -0.4);
            while (true) { // turn the 180
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (angle < 0) {
                    break;
                }
                turnLeft(0.4);
                if (angle < 0) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            odo.resetPosAndIMU();
            sleep(500);
            while (true) { // grab the block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().x >= 200) {
                    break;
                }
                driveForwardCorrection(angle, 0.5, 200, pos.getPosition().x);
                if (pos.getPosition().x >= 200) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            while (true) { // back up with said block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.4, 250, pos.getPosition().x);
                if (pos.getPosition().x <= 100) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            odo.resetPosAndIMU();
            sleep(500);
            stopRobot();
            while (true) { // strafe back to the hanging thing
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().y <= 100) {
                    break;
                }
                strafeRight(0.4);
                if (pos.getPosition().y <= 100) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            turnDegrees(10, 0.4);
            while (true) { // turn to face said hanging thing
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (angle > 0) {
                    break;
                }
                turnRight(0.4);
                if (angle > 0) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            while (true) { // put the sample on the hanging thing
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (pos.getPosition().x >= 400) {
                    break;
                }
                driveForwardCorrection(angle, 0.5, 400, pos.getPosition().x);
                if (pos.getPosition().x >= 400) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            sleep(3000);
            telemetry.update();
            while (true) { // back up from said hanging thing
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.4, 0, pos.getPosition().x);
                if (pos.getPosition().x <= 0) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();

//            odo.resetPosAndIMU();
//            sleep(1500);
//            while (true) {
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.6, -10, pos.getPosition().x);
//                if (pos.getPosition().x <= -10) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            sleep(333);
//            while (true) {
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                strafeRight(0.4);
//                if (pos.getPosition().y >= 900) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            sleep(333);
//            turnDegrees(160, -0.6);
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            sleep(333);
//            while (true) {
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveForwardCorrection(angle, 0.6, 60, pos.getPosition().x);
//                if (pos.getPosition().x >= 80) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            while (angle >115){
//                    Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
//                    String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                    telemetry.addData("Position", data);
//                    angle = (angleWrap(odo.getHeading()));
//                    odo.bulkUpdate();;
//                    turnLeft(0.2);
//                    telemetry.update();
//            }
//            stopRobot();
//            odo.resetPosAndIMU();
//            sleep(3000);
//            while (true) {
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.6, -60, pos.getPosition().x);
//                if (pos.getPosition().x <= -20) {
//                    break;
//                }
//                telemetry.update();
//            }
            }
            //DRIVE_DISTANCE_FORWARD(28,1.2);
            //DRIVE_DISTANCE_LEFT(20.4f);
            //DRIVE_DISTANCE_FORWARD(-24,1.2);
            //TURN(1,20);

            //move arm to place pixel
        }



    }