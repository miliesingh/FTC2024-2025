package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;                //imports from FIRST
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@TeleOp
public class doubledriver extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;                                         //Declaring Motor varibles 
    private DcMotor backLeft;
    private DcMotor backRight;



    private CRServo Left;
    private DcMotor Crain;
    private DcMotor Spin;

    private Rev2mDistanceSensor distance;


    public void runOpMode() throws InterruptedException {



        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");                            //mapping motors from control hub 
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        Left = hardwareMap.get(CRServo.class, "Lefts");
        Crain = hardwareMap.get(DcMotor.class, "Crane");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

       distance = hardwareMap.get(Rev2mDistanceSensor.class,"distance");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);                 //setting direction of drive train 
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        Crain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Crain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            float pickup;                                   //setting varibles from conteroler imputs  
            float dropoff;
            boolean spinpowerup;
            boolean spinpowerdown;
            double crainpower;
            boolean spincenter;
            boolean opspincenter;
            boolean burst;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;

            crainpower = gamepad2.right_stick_y;
            spinpowerup = gamepad2.dpad_right;
            spincenter = gamepad2.y;
            opspincenter = gamepad2.x;
            spinpowerdown =gamepad2.dpad_left;
            pickup = gamepad2.left_trigger;
            dropoff = gamepad2.right_trigger;
            burst = gamepad2.b;


            if (strafeRight) {
                frontLeft.setPower(-.8);
                frontRight.setPower(1);                         //conecting motor varibles to controler inputs
                backLeft.setPower(1);
                backRight.setPower(-1);
            }
            if (strafeLeft) {
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backLeft.setPower(-.8);
                backRight.setPower(1);
            }


            frontLeft.setPower(throttle*.91);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle*.91);
            backRight.setPower(throttle);

            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);

            Crain.setPower(crainpower);

            if (spinpowerup){
                Spin.setPower(1);
            }
            if (spinpowerdown){
                Spin.setPower(-1);
            }

            if (!spinpowerdown && !spinpowerup ){
                Spin.setPower(0);
            }

            if (spincenter){
                Spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Spin.setTargetPosition(-20);
                Spin.setPower(1);
                Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1500);
                Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (opspincenter){
                Spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Spin.setTargetPosition(-589);
                Spin.setPower(1);
                Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1500);
                Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (pickup>0) {

                Left.setPower(-1);
            }

            if (dropoff>0){

                Left.setPower(1);

            }

            if (dropoff == 0 && pickup == 0){

                Left.setPower(0);

            }
            /*
            while (burst) {
                Left.setPower(.8);
                sleep(500);
                Left.setPower(0);
                sleep(1500);
                if (burst == false) {
                    break;
                }
            }
            */
            //telemetry.addData("encoder value", Crain.getCurrentPosition());
            //telemetry.update();


           //telemetry.addData("distance",distance.getDistance(DistanceUnit.INCH));
            //telemetry.update();
        }


    }
}
