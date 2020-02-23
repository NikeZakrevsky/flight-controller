#include <Wire.h>
#include <Servo.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

char data;
long acc_x, acc_y, acc_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int gyro_x, gyro_y, gyro_z;
int temperature;
float angle_pitch, angle_roll, angel_yaw;
long loop_timer;
boolean set_angles;
float acc_total_vector, angle_pitch_acc, angle_roll_acc, angle_pitch_output, angle_roll_output;
float escA, escB, escC, escD;
float throttle = 1300;
boolean stop = true;
float desired_angel_pitch, desired_angel_roll;
float error_pitch, error_roll;

float p_gain = 0;
float i_gain = 0;
float d_gain = 3;

float last_pitch_error, last_roll_error, pitch_pid, roll_pid;
float pitch_p, pitch_i, pitch_d, roll_p, roll_i, roll_d;

float elapsedTime, time, timePrev;

Servo motA, motB, motC, motD;

void setup() {
  Wire.begin();
  Serial.begin(57600);
  setup_mpu();
  Serial.print("Calibrating...");
  for (int i = 0; i < 200; i++) {
    Serial.println(i);
    read_mpu_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_x_cal /= 200;
  gyro_y_cal /= 200;
  gyro_z_cal /= 200;

  Serial.print("Finish.");

  motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    
  motA.writeMicroseconds(MIN_PULSE_LENGTH);
  motB.writeMicroseconds(MIN_PULSE_LENGTH);
  motC.writeMicroseconds(MIN_PULSE_LENGTH);
  motD.writeMicroseconds(MIN_PULSE_LENGTH);

  loop_timer = micros();
}

void loop() {
  timePrev = time;
  time = micros();
  elapsedTime = (time - timePrev) / 1000000.0f;
  
  if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : {
                      Serial.println("Sending minimum throttle");
                      motA.writeMicroseconds(MIN_PULSE_LENGTH);
                      motB.writeMicroseconds(MIN_PULSE_LENGTH);
                      motC.writeMicroseconds(MIN_PULSE_LENGTH);
                      motD.writeMicroseconds(MIN_PULSE_LENGTH);
            }
            break;

            // 1
            case 49 : {
                    Serial.println("Sending maximum throttle");
                      motA.writeMicroseconds(MAX_PULSE_LENGTH);
                      motB.writeMicroseconds(MAX_PULSE_LENGTH);
                      motC.writeMicroseconds(MAX_PULSE_LENGTH);
                      motD.writeMicroseconds(MAX_PULSE_LENGTH);
            }
            break;

            //2
            case 50 : {
                      p_gain -= 0.1;
                      Serial.println(p_gain);
            }
            break;
            //3
            case 51 : {
                      p_gain += 0.1;
                      Serial.println(p_gain);
            }
            break;
     
            // 4
            case 52 : {
                      d_gain -= 0.1;
                      Serial.println(d_gain);
            }
            break;

            // 5
            case 53 : {
                      d_gain += 1;
                      Serial.println(d_gain);
            }
            break;

            // 7
            case 55 : {
                      i_gain -= 0.001;
                      Serial.println(i_gain);
            }
            break;

            // 8
            case 56 : {
                      i_gain += 0.001;
                      Serial.println(i_gain);
            }
            break;


            
            // 9
            case 57 : stop = !stop;
            break;
        }
    }
    
  read_mpu_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;


  angle_pitch += gyro_y / 65.5 * elapsedTime;
  angle_roll += gyro_x / 65.5 * elapsedTime;


  //angle_pitch -= angle_roll * sin(gyro_z / 65.5 * 3.14 / 180);
  //angle_roll += angle_pitch * sin(gyro_z / 65.5 * 3.14 / 180);
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  angle_pitch_acc = asin((float)acc_x/acc_total_vector) * 57.296;
  angle_roll_acc = asin((float)acc_y/acc_total_vector) * -57.296;
  
  angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;
  angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02; 
  
  // PITCH PID
  error_pitch = angle_pitch - desired_angel_pitch;
  if (-3 < error_pitch < 3) {
    pitch_i = pitch_i + (error_pitch * i_gain);
  }

  pitch_p = error_pitch * p_gain;
  pitch_d = d_gain * ((error_pitch - last_pitch_error) / elapsedTime);
  
  pitch_pid = pitch_p + pitch_i + pitch_d;
  last_pitch_error = error_pitch;


  //ROLL PID
  error_roll = angle_roll - desired_angel_roll;
  if (-3 < error_roll < 3) {
    roll_i = roll_i + (error_roll * i_gain);
  }

  roll_p = error_roll * p_gain;
  roll_d = d_gain * ((error_roll - last_roll_error) / elapsedTime);
  
  roll_pid = roll_p + roll_i + roll_d;
  last_roll_error = error_roll;

  if(roll_pid < -400){roll_pid=-400;}
  if(roll_pid > 400) {roll_pid=400; }
  if(pitch_pid < -4000){pitch_pid=-400;}
  if(pitch_pid > 400) {pitch_pid=400;}
  
  escA = throttle - pitch_pid + roll_pid;
  escB = throttle + pitch_pid + roll_pid;
  escC = throttle - pitch_pid - roll_pid;
  escD = throttle + pitch_pid - roll_pid;

  if(escA < 1100){escA=1100;}
  if(escA > 2000) {escA=2000; }

  if(escB < 1100){escB=1100;}
  if(escB > 2000) {escB=2000; }

  if(escC < 1100){escC=1100;}
  if(escC > 2000) {escC=2000; }

  if(escD < 1100){escD=1100;}
  if(escD > 2000) {escD=2000; }
  

  if (escA > 1800) escA = 1800;
  if (escB > 1800) escB = 1800;
  if (escC > 1800) escC = 1800;
  if (escD > 1800) escD = 1800;

  if (escA < 1300) escA = 1300;
  if (escB < 1300) escB = 1300;
  if (escC < 1300) escC = 1300;
  if (escD < 1300) escD = 1300;

  
  if (!stop) {
    motA.writeMicroseconds(escA);
    motB.writeMicroseconds(escB);
    motC.writeMicroseconds(escC);
    motD.writeMicroseconds(escD);
  } else {
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
    motC.writeMicroseconds(MIN_PULSE_LENGTH);
    motD.writeMicroseconds(MIN_PULSE_LENGTH);
  }

  
}

void setup_mpu() {
  Wire.beginTransmission(0x68);       
  Wire.write(0x6B);                                                   
  Wire.write(0x00);                                                  
  Wire.endTransmission();    
                    
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                               
  Wire.endTransmission();
                                              
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                   
  Wire.endTransmission();    
}

void read_mpu_data() {
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                   
  Wire.endTransmission();                                              
  Wire.requestFrom(0x68,14);                                           
  while(Wire.available() < 14);                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();
  acc_z *= -1;
  temperature = Wire.read()<<8|Wire.read();                            
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();    
  gyro_z = Wire.read()<<8|Wire.read();
}
