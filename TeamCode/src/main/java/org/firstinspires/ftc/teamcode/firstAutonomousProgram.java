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
@Autonomous(name="testAutonomousProgramBlueSideSpecimen", group="Auto2024")
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
        backLeft.setPower(power/2);
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
        if (pos < distance / 2) {
            if (angle <= -5) {
                softTurnLeft(power / 2);
                turn = "left";
            } else if (angle >= 5) {
                softTurnRight(power / 2);
                turn = "right";
            } else if (angle < 5 && angle > -5) {
                driveForward(power / 2);
                turn = "forward";
            } else {
                stopRobot();
            }
        }
        if (pos > distance / 2) {
            if (angle <= -5) {
                softTurnLeft(power);
                turn = "left";
            } else if (angle >= 5) {
                softTurnRight(power);
                turn = "right";
            } else if (angle < 5 && angle > -5) {
                driveForward(power);
                turn = "forward";
            } else {
                stopRobot();
            }
        }

        telemetry.addData("turn", turn);
        telemetry.update();
    }
    public void driveForwardCorrection(double angle, double power, int distance, double pos){
        String turn = "";
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        if (pos>distance/2){
            if(angle <=-5){
                softTurnLeft(power/2);
                turn = "left";
            }
            else if (angle>= 5) {
                softTurnRight(power/2);
                turn = "right";
            }
            else if (angle <5 && angle > -5) {
                driveForward(power/2);
                turn = "forward";
            }
            else{
                stopRobot();
            }
        }
        if (pos<distance/2){
            if(angle <=-5){
                softTurnLeft(power);
                turn = "left";
            }
            else if (angle>= 5) {
                softTurnRight(power);
                turn = "right";
            }
            else if (angle <5 && angle > -5) {
                driveForward(power);
                turn = "forward";
            }
            else{
                stopRobot();
            }
        }

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

////            driveDistanceForward(0.5, -500, pos.getPosition().x);
//            driveForward(0.2, angleWrap(odo.getHeading()));
            while (true) {
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.4, 711, pos.getPosition().x);
                if(pos.getPosition().x > 711) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            sleep(250);
            odo.resetPosAndIMU();
            while (true) {
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.4, -203, pos.getPosition().x);
                if(pos.getPosition().x < -203) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            //DRIVE_DISTANCE_FORWARD(28,1.2);
            //DRIVE_DISTANCE_LEFT(20.4f);
            //DRIVE_DISTANCE_FORWARD(-24,1.2);
            //TURN(1,20);

            //move arm to place pixel
        }



    }
}
