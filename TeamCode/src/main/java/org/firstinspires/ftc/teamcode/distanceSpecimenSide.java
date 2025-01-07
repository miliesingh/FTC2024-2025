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

// This is for the specimen side
@Autonomous(name="distanceFunctionSpecimenSide", group="Auto2024")
public class distanceSpecimenSide extends LinearOpMode {
    // setting the objects
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private Servo clawWristServo = null;
    private Servo clawServo = null;

    private DistanceSensor sensorRange;

    // setting the odometry object
    GoBildaPinpointDriver odo;

    //below is the linear slider function - only problem is that it has to go one step at a time
    public void LINEAR_SLIDE_DRIVE(float distance_in_in, double power) {
        float ticksPerInch = 450.149432158f;
        float f_ticks = ticksPerInch * distance_in_in;
        int ticks = Math.round(f_ticks);
        //753.1 ticks per revolution
        //1.673 in per revolution (circumference)
        //450.149432158 ticks per in
        if (power > 0) {
            //go up
            linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
            linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            //go down
            linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
            linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setPower(power);
        linSlideL.setTargetPosition(ticks);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setPower(power);
        linSlideR.setTargetPosition(ticks);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("slide", "> is running to position");
        telemetry.update();
        while ((linSlideR.getCurrentPosition() <= linSlideR.getTargetPosition() - 50) && linSlideL.getCurrentPosition() <= (linSlideL.getTargetPosition() - 50)) {
            //Wait until job is finished
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" +
                    linSlideR.getCurrentPosition() + " need to get to " +
                    linSlideR.getTargetPosition());
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" + linSlideL.getCurrentPosition() + " need to get to " + linSlideL.getTargetPosition());
            telemetry.update();
        }
    }
    // this function turns the heading given by the odometry into angles - it was in radians
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        double angle = radians * 180 / Math.PI;
        // keep in mind that the result is in radians
        return angle;
    }
    //simple drive forward function
    public void driveForwards(double power){
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
    }
    // this function goes forward at a certain power to a set distance away from something
    public void controlDistance(double distance, double power){
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        double theDistance = sensorRange.getDistance(DistanceUnit.CM);
        // getting the distance
        while (theDistance > distance){
            // while it is out of distance, move forward
            theDistance = sensorRange.getDistance(DistanceUnit.CM);
            driveForwards(power);
            telemetry.update();
        }
        stopRobot();
    }
    //function to turn left - sets power and direction
    public void turnLeft(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    //function to strafe right - sets direction of motors and power
    public void strafeRight(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    // this soft turn left basically allows for turning while driving

    public void softTurnLeft(double power) {
        leftBack.setPower(power / 2);
        rightFront.setPower(power);
        leftFront.setPower(power / 2);
        rightBack.setPower(power);
    }

    //same thing - allows for turning while driving

    public void softTurnRight(double power) {
        rightBack.setPower(power / 2);
        leftFront.setPower(power);
        rightFront.setPower(power / 2);
        leftBack.setPower(power);
    }

    // turning right - sets direction of motors and the power
    public void turnRight(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    // the all inclusive stoping the robot
    public void stopRobot() {
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }
    // this is the other drive forward function - it doesn't set the power - i kept it because there are other functions that use this function
    public void driveForward(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
    }
    // at this point this just drives backward
    public void driveBackwardCorrection(double angle, double power, int distance, double pos) {
        String turn = "";
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        driveForward(power);
        telemetry.addData("turn", turn);
        telemetry.update();
    }
    // also just drives forward
    public void driveForwardCorrection(double angle, double power, int distance, double pos) {
        String turn = "";
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveForward(-power);

        telemetry.addData("turn", turn);
        telemetry.update();
    }
    // this function turns the robot x amount of degrees - negative power for turning left - but new angle is always positive
    public void turnDegrees(double nAngle, double power) {
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            double angle = (angleWrap(odo.getHeading()));
            //getting the angle of the robot
            odo.bulkUpdate();
            turnRight(power);
            if (Math.abs(angle) > nAngle) {
                // if the abs value of the angle of the robot is larger than the new angle then it stops
                break;
            }
            telemetry.update();
        }
        stopRobot();
    }

    // this function basically slows down once the robot reaches halfway of the distance for more accuracy
    public void driveDistanceForward(double power, int distance, double xPlace) {
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        if (xPlace > distance) {
            // if the distance is greater than move full power
            if (xPlace > distance / 2) {
                leftBack.setPower(power);
                rightFront.setPower(power);
                leftFront.setPower(power);
                rightBack.setPower(power);
                telemetry.update();
            }
            // if the distance is half move at half power
            if (xPlace < distance / 2) {
                leftBack.setPower(power / 2);
                rightFront.setPower(power / 2);
                leftFront.setPower(power / 2);
                rightBack.setPower(power / 2);
                telemetry.update();
            }
        } else {
            // stopping the robot
            leftBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
        }
    }
    // there is a lot of functions because we need like 4 for each range of motion
    public void driveForwardXIncrease(double power, int newPos) {
        double angle;
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            // putting the telemetry up
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            // driving forward until the x position is greater
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void driveBackwardXDecrease(double power, int newPos) {
        double angle;
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            // telemetry
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveBackwardCorrection(angle, -power, 590, pos.getPosition().x);
            // driving backward until the x position is less
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void strafeRightYIncrease(double power, int newPos) {
        double angle;
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y >= newPos) {
                break;
            }
            // its in there twice because it needs to check before and after
            strafeRight(power);
            // strafing until the y position is greater
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void strafeLeftYDecrease(double power, int newPos){
        double angle;
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // adding the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y <= newPos) {
                break;
            }
            // strafing until the y position is less
            strafeRight(-power);
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // this function is only for if the robot is off to the right by a bit - and if it is facing forward
    public void angleCorrectionFacingZeroRight(double power) {
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // adding telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < 2) {
                break;
            }
            // turning right until the angle is within the range
            turnRight(power);
            if (angle < 2) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
    }
    // this function doesn't distinguish between if the robot is facing left or right
    public void angleCorrectionFacingZeroBothSides(double power){
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0) {
                // if the robot is facing left

                if (angle < 2) {
                    break;
                }
                turnRight(0.2);
                //turning right until the angle is right
                if (angle < 2) {
                    break;
                }

            } else if (angle < 0) {
                // if the robot is facing right
                if (angle > -2) {
                    break;
                }
                // turning left until the angle is right
                turnLeft(0.2);
                if (angle > -2) {
                    break;
                }
            } else {
                break;
            }
        }
        stopRobot();
        telemetry.update();
    }
    // this just turning right until the angle is 180
    public void turnAroundRightZeroTo180(double power) {
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < -172) {
                break;
            }
            // turning until the angle is equal to 180
            turnRight(power);
            if (angle < -172) {
                break;
            }
            telemetry.update();
        }


        stopRobot();
    }
    // doesn't distinguish between the 2 sides for robot effiency
    public void turnAroundLogic180ToZero(double power) {
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0) {
                // if the robot is facing left

                if (angle < 7) {
                    break;
                }
                // turning until the robot is 180 degrees turned
                turnRight(power);
                if (angle < 7) {
                    break;
                }

            } else if (angle < 0) {
                // if the robot is facing right
                if (angle > -7) {
                    break;
                }
                // turning left until the angle is correct
                turnLeft(power);
                if (angle > -7) {
                    break;
                }
            } else {
                break;
            }
        }
        telemetry.update();
        stopRobot();
    }
    // back to the batch of functions for movement
    public void driveForwardXDecrease(double power, int newPos) {
        double angle;
        while (true) { // getting the block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // the new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 400, pos.getPosition().x);
            // driving forward until the condition is met
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();


        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linSlideL = hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        linSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor");

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

        clawServo.setPosition(Servo.MAX_POSITION);
        odo.resetPosAndIMU();
        double angle = angleWrap(odo.getHeading());
        if (opModeIsActive()) {
            clawServo.setPosition(Servo.MIN_POSITION);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            odo.bulkUpdate();
            sleep(333);
            LINEAR_SLIDE_DRIVE(8f, 0.9);
            driveForwardXIncrease(0.4, 550); // going up to hang the specimen
            LINEAR_SLIDE_DRIVE(3f, -0.7); // hanging the specimen
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            driveBackwardXDecrease(0.2, 549); // backing up from hanging
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
            telemetry.update();
            sleep(333);
            strafeRightYIncrease(0.6, 650);// strafing right to the first block
            clawWristServo.setPosition(1.0);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            angleCorrectionFacingZeroRight(0.2);
            driveForwardXIncrease(0.6, 1150); // driving over the first block
            clawWristServo.setPosition(0.0);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            strafeRightYIncrease(0.6, 900); // strafing over the first block
            clawWristServo.setPosition(0.0);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            driveBackwardXDecrease(0.7, 380); // pushing the first block into the human player zone
            driveForwardXIncrease(0.4, 500); // going out of the human player zone
            turnAroundRightZeroTo180(0.4); // turning around
            sleep(500);
            telemetry.update();
            clawServo.setPosition(1); // opening the claw
            sleep(1000);
            driveForwardXDecrease(0.4, -100); // grabbing the block
            clawServo.setPosition(0.0); // grabbing block
            sleep(500);
            clawWristServo.setPosition(1.0);// putting the thing up
            turnAroundLogic180ToZero(0.6);
            strafeLeftYDecrease(0.8, -50); // going back over to hang the block
            angleCorrectionFacingZeroBothSides(0.2);
            LINEAR_SLIDE_DRIVE(8f, 1);
            controlDistance(32, 0.4); // going forward to hang the block
            LINEAR_SLIDE_DRIVE(3f, -0.7); // hanging the specimen
            clawServo.setPosition(Servo.MAX_POSITION);
            sleep(500);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            driveBackwardXDecrease(0.2, 559); // backing up from hanging
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
        }

    }
}