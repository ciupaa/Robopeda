package pedroPathing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;


@TeleOp(name = "TeleOpp", group = "Robot")
//@Disabled
public class TeleOpp extends LinearOpMode {

    public DcMotor  fata_stanga   = null; //the left drivetrain motor
    public DcMotor  fata_dreapta  = null; //the right drivetrain motor
    public DcMotor  spate_stanga    = null;
    public DcMotor  spate_dreapta   = null;
    public DcMotor  motor_stanga         = null; //the arm motor
    public DcMotor  motor_glisiere        = null; //
    public Servo  servoRotire           = null; //the active servoRotire servo
    public Servo    cleste            = null; //the cleste servo
    public DcMotor hang1 = null;
    public DcMotor hang2 = null;


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 71.2 // This is the exact gear ratio of the gobilda 60rpm motor
                    * 9.6 // This is the external gear reduction
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double cosSusBrat = 115 * ARM_TICKS_PER_DEGREE;
    final double cosJosBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double hang = 140 * ARM_TICKS_PER_DEGREE;
    final double intake = 25 * ARM_TICKS_PER_DEGREE;
    final double servoRetras = 0.4;
    final double servoTras = 0.7;

    /* Variables to store the positions that the cleste should be set to when folding in, or folding out. */
    final double cleste_inchis   = 0;
    final double cleste_deschis  = 1;


    /* A number in degrees that the triggers can adjust the arm position by */


    /* Variables that are used to set the arm to a specific position */
    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE;
    final double miscareHang = 0;
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;

    double armPositionFudgeFactor;



    final double LIFT_TICKS_PER_MM = 384.5 / 120.0; // Encoder ticks per mm for your specific motor and pulley setup
    // final double LIFT_TICKS_PER_MM = 537.6 / 120.0; // Approximately 4.48


    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 100 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 470 * LIFT_TICKS_PER_MM;
    final double LIFT_LIMIT = 470 * LIFT_TICKS_PER_MM;
    double liftPosition = LIFT_COLLAPSED;

    final double LIFT_MAX_POSITION = (int) (470 * LIFT_TICKS_PER_MM); // Fully extended for 240mm slide



    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        /* Define and Initialize Motors */
        fata_stanga   = hardwareMap.get(DcMotor.class, "fata_stanga"); //the arm motor
        spate_stanga   = hardwareMap.get(DcMotor.class, "spate_stanga"); //the arm motor
        fata_dreapta   = hardwareMap.get(DcMotor.class, "fata_dreapta"); //the arm motor
        spate_dreapta   = hardwareMap.get(DcMotor.class, "spate_dreapta"); //the arm motor
        motor_glisiere = hardwareMap.dcMotor.get("motor_glisiere");
        motor_stanga   = hardwareMap.get(DcMotor.class, "motor_stanga"); //the arm motor
        hang1 = hardwareMap.get(DcMotor.class, "hang1");
        hang2 = hardwareMap.get(DcMotor.class, "hang2");

       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */

        fata_stanga.setDirection(DcMotor.Direction.REVERSE);
        spate_stanga.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);
        hang1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);
        hang2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior. BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) motor_stanga).setCurrentAlert(5,CurrentUnit.AMPS);

        ((DcMotorEx) motor_glisiere).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the motor_stanga. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor_stanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_stanga.setTargetPosition(0);
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setTargetPosition(0);
        motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_glisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");
        cleste  = hardwareMap.get(Servo.class, "cleste");

        /* Make sure that the servoRotire is off, and the cleste is folded in. */
        //  servoRotire.setPower(rotirePosition);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {
            // Rotate the movement direction counter to the bot's rotation
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hang1.setPower(-gamepad2.left_stick_y);
            hang2.setPower(-gamepad2.left_stick_y);

            motor_stanga.setPower(-gamepad2.right_stick_y);
            motor_glisiere.setPower(gamepad2.right_stick_x);


            fata_stanga.setPower(frontLeftPower);
            spate_stanga.setPower(backLeftPower);
            fata_dreapta.setPower(frontRightPower);
            spate_dreapta.setPower(backRightPower);

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            servoRotire and cleste) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the cleste to make sure it is in the correct orientation to servoRotire, and it
            turns the servoRotire on to the COLLECT mode.*/
            if (gamepad2.x) {
                cleste.setPosition(0.6);
            }
            else if (gamepad2.a)
                cleste.setPosition(1);
            if(gamepad2.b)
                servoRotire.setPosition(0.4);
            else if (gamepad2.y)
                servoRotire.setPosition(0.6);

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
            // armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            if(gamepad2.dpad_up) {
                armPosition = cosSusBrat;
                liftPosition += LIFT_SCORING_IN_HIGH_BASKET * cycletime;
            }
            if(gamepad2.dpad_down) {
                armPosition = cosJosBrat;
                liftPosition = LIFT_SCORING_IN_LOW_BASKET * cycletime;
            }
            if (gamepad2.dpad_left) {
                armPosition = intake;
                liftPosition = LIFT_COLLAPSED;
            }
            if (gamepad2.right_stick_button)
                armPosition = hang;


            if(gamepad1.a)
                armPosition = hang + 15 * ARM_TICKS_PER_DEGREE;

            if(gamepad1.b)
                armPosition = ARM_COLLAPSED_INTO_ROBOT;


            if(gamepad2.dpad_right) {
                liftPosition = LIFT_COLLAPSED;
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
            }


// Apply position limits
            if (armPosition < ARM_COLLAPSED_INTO_ROBOT) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT; // Clamp to min position
            }

// Set target position for the motor
            motor_stanga.setTargetPosition((int) (armPosition + armPositionFudgeFactor ));

// Set motor velocity (max speed)
            ((DcMotorEx) motor_stanga).setVelocity(4500); // Maximum velocity
            motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// Check for overcurrent condition and report via telemetry
            if (((DcMotorEx) motor_stanga).isOverCurrent()) {
                telemetry.addLine("BRAT EXCEEDED CURRENT LIMIT!");
            }
            if (((DcMotorEx) motor_glisiere).isOverCurrent()) {
                telemetry.addLine("GLISIERE EXCEEDED CURRENT LIMIT!");
            }


// Update lift position based on driver input
            if (gamepad2.right_bumper) {
                liftPosition += 2000 * cycletime; // Increased for faster movement
            } else if (gamepad2.left_bumper) {
                liftPosition -= 2000 * cycletime; // Increased for faster movement
            }



// Enforce limits
            // Enforce limits

            if (armPosition < 45 * ARM_TICKS_PER_DEGREE) {
                if ( liftPosition > LIFT_LIMIT) {
                    liftPosition = LIFT_LIMIT;
                }
            }

            if (liftPosition > LIFT_MAX_POSITION) {
                liftPosition = LIFT_MAX_POSITION;
            }
            else if (liftPosition < LIFT_COLLAPSED) {
                liftPosition = LIFT_COLLAPSED;
            }

// Set motor mode and target position
            motor_glisiere.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Ensure it's in the correct mode
            motor_glisiere.setTargetPosition((int) liftPosition);

// Set motor velocity (ticks per second)
            ((DcMotorEx) motor_glisiere).setVelocity(3200); // Adjust for desired speed



            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", motor_stanga.getTargetPosition());
            telemetry.addData("arm Encoder: ", motor_stanga.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",motor_glisiere.getTargetPosition());
            telemetry.addData("lift current position", motor_glisiere.getCurrentPosition());
            telemetry.addData("motor_glisiere Current:",((DcMotorEx) motor_glisiere).getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}