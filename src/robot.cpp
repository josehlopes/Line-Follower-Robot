#include <Ultrasonic.h>

#define Trigger 13
#define Echo 12
#define MotorL 3
#define MotorR 4
#define SensorL A0
#define SensorR A1
#define BLACK_THRESHOLD 600

Ultrasonic ultrasonic(Trigger, Echo);

class PID
{
public:
    PID(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0), integral(0) {}

    float calculate(float setpoint, float measured_value)
    {
        float error = setpoint - measured_value;
        integral += error;
        float derivative = error - prev_error;
        prev_error = error;
        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

private:
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
};

PID pid(1.0, 0.1, 0.01);

void Accel()
{
    digitalWrite(MotorL, HIGH);
    digitalWrite(MotorR, HIGH);
}

void Stop()
{
    digitalWrite(MotorL, LOW);
    digitalWrite(MotorR, LOW);
}

void TurnR()
{
    digitalWrite(MotorL, HIGH);
    digitalWrite(MotorR, LOW);
}

void TurnL()
{
    digitalWrite(MotorL, LOW);
    digitalWrite(MotorR, HIGH);
}

void Dodge()
{
    Stop();
    delay(500);
    TurnR();
    delay(200);
    Stop();
    delay(500);
    Accel();
    delay(500);
    Stop();
    delay(500);
    TurnL();
    delay(400);
    Stop();
    delay(500);
    Accel();
    delay(440);
    TurnR(); //teste
    delay(200);
    FollowLine();
}

bool CheckObstacle()
{
    int distance = ultrasonic.read();
    delayMicroseconds(20);

    return (distance <= 25);
}

bool isOnBlackLine(int sensorValue)
{
    return sensorValue > BLACK_THRESHOLD;
}

void FollowLine()
{
    int sensorL_value = analogRead(SensorL);
    int sensorR_value = analogRead(SensorR);

    bool isLeftBlack = isOnBlackLine(sensorL_value);
    bool isRightBlack = isOnBlackLine(sensorR_value);

    float setpoint = 0;
    float error = sensorL_value - sensorR_value;
    float control_signal = pid.calculate(setpoint, error);

    bool obstacle = CheckObstacle();
    if (obstacle)
    {
        Dodge();
    }
    else
    {
        if (isLeftBlack && isRightBlack)
        {
            float correction = pid.calculate(0, error);
            if (correction > 0)
            {
                TurnR();
                Serial.println("Correção para a direita");
            }
            else if (correction < 0)
            {
                TurnL();
                Serial.println("Correção para a esquerda");
            }
            else
            {
                Accel();
                Serial.println("Correção Acelerar");
            }
        }
        else if (isLeftBlack)
        {
            TurnL();
            Serial.println("Virando para a esquerda");
        }
        else if (isRightBlack)
        {
            TurnR();
            Serial.println("Virando para a direita");
            
        }
        else
        {
            Accel();
            Serial.println("Acelerando");
        }
    }

    delay(20);
}

void setup()
{
    Serial.begin(9600);
    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(SensorL, INPUT);
    pinMode(SensorR, INPUT);
    pinMode(MotorL, OUTPUT);
    pinMode(MotorR, OUTPUT);
}

void loop()
{
    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(2);
    digitalWrite(Trigger, LOW);
    FollowLine();
}