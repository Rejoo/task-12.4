#define ENABLE 9                            
#define INT1   2                            
#define INT2   3                            
#define FEEDBACK_PIN A0                     //reading motor speed feedback 

double alpha = 0.8;        //smothing factor
double smoothed_value = 0; 

double SetPoint_speed = 150;    

// PID Controller class
class PIDController {
private:
    double Kp;
    double Ki;
    double Kd;
    double Integral;
    double Previous_Error;
    double Last_Time;

public:
    PIDController(double kp, double ki, double kd) 
        : Kp(kp), Ki(ki), Kd(kd), Integral(0), Previous_Error(0), Last_Time(millis()) {}

    double Compute(double setPoint, double actualValue) { //calculating PID OUtPUT
        double Time_Now = millis();
        double dt = (Time_Now - Last_Time) / 1000.0;  
        Last_Time = Time_Now;

        double Error = setPoint - actualValue;

        Integral += Error * dt;                                                            
        double Derivative = (Error - Previous_Error) / dt;                                
        Previous_Error = Error;                                                           

        double PID_OUTPUT = (Kp * Error) + (Ki * Integral) + (Kd * Derivative); // Calculate PID output
        return constrain(PID_OUTPUT , 0, 255);
    }
};

PIDController pid(0.8, 20, 0.001); // Initialize PID Controller

void setup() {
    Serial.begin(115200);

 
    pinMode(ENABLE, OUTPUT);
    pinMode(INT1, OUTPUT);
    pinMode(INT2, OUTPUT);


    digitalWrite(INT1, LOW);
    digitalWrite(INT2, HIGH);

    analogWrite(ENABLE, 0);

    // Initialize smoothed_value with the first reading
    smoothed_value = map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255);
}

void loop() {
    double Actual_Value = map(analogRead(FEEDBACK_PIN), 0, 1023, 0, 255);  // Scaling 0-1023 to 0-255

    smoothed_value = ExponentialSmoothing(Actual_Value, smoothed_value, alpha);

    double OutPut = pid.Compute(SetPoint_speed, smoothed_value);

    analogWrite(ENABLE, (int)OutPut);

    Serial.print("Set Point: "); Serial.print(SetPoint_speed);
    Serial.print("\tActual Value: "); Serial.print(smoothed_value);
    Serial.print("\tOutput (PWM): "); Serial.println(OutPut);

    delay(1000);       
}

double ExponentialSmoothing(double input, double previous_smoothed, double alpha) {
    return alpha * previous_smoothed + (1 - alpha) * input ;
}

```