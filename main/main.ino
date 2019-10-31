// #include <Servo.h>

// Constants:
float g = 9.81; //[m/s^2]

// Calculate F_F_max from MATLAB approximation with final variables:
// Leave as const for now, move to be calculated within Arduino later
float F_F_max = 336.34; //[N]

// Define the potentiometer position corresponding to a maximum braking force [0 1023]
int p_i_max = 950; 

float mu_s = 0.4; // Similar to tire rubber on grass (underestimated for normal cycling conditions)

// x,y,z components of distance from C1 to COM
float d_C1_COM[3] = {0.5, 1, 0}; //[m]
//malloc(sizeof(float)*3)
//d_C1_COM[0] = 0.5; d_C1_COM[1] = 1; d_C1_COM[2] = 0; 

// x,y,z components of distance from C1 to C2
float d_C1_C2[3] = {1.12, 0, 0}; //[m]

float M = 80; //[kg]

float SB1 = 0; 

float R = 0.66/2; //[m]

float r = 0.16/2; //[m]

float I_A2 = 0.2*0.2*2; //[kg*m^2]

float d_C2_COM[3] = {0.7, 1, 0}; //[m]

float SB2 = 0; 

float I_A1 = 0.2*0.2*2; //[kg*m^2]

float d_A1_COM[3] = {0.3, 0.6, 0}; //[m]

// Read input potentiometer value
int ReadPot(){
  int p_i = 500;
  return p_i;
}

// Read the input angle of incline (in radians)
float ReadGyro(){
  float theta = 0.1; //[rad]
  return theta;
}

float DesiredGroundFriction(float F_F_max, int p_i_max, int p_i){
  float F_F_desired = min(F_F_max, F_F_max*p_i/p_i_max);
  return F_F_desired;
}

float MaximumGroundFriction(float mu_s,float* d_C1_COM, float M, float theta, float* d_C1_C2){
  //   d_C1_COM is a 3D vector of the x, y, z components of the displacement from C1 to COM. 
  //   theta is the angle of incline in radians 
  //   d_C1_C2 is just the x-component since it is defined to point in the x-direction

  float F_N = M*g*(d_C1_COM[0]*cos(theta)-d_C1_COM[1]*sin(theta))/d_C1_C2[0];
  float F_F_max = mu_s*F_N;
  return F_F_max;
}

float ProvidedDiskBraking(float M, float R, float r, float I_A2, float F_F_desired, float F_F_provided){
  float F_b_provided = (F_F_provided*R - I_A2*F_F_desired/(R*M))/r;
  return F_b_provided;
}

float MaximumGroundFrictionNoseOver(float M, float theta,float* d_A1_COM){
  // Assume a_z=0
  // Assume back tire never leaves the ground (but normal force is zero, therefore friction is zero)
  // Assume no acceleration about front axle
  
  // Using hungarian equation:
  float a_x = g*(sin(theta)*d_A1_COM[1]-cos(theta)*d_A1_COM[0])/d_A1_COM[1];
  float F_F1_NO = -M*(a_x-g*sin(theta));
  return F_F1_NO;
}

void RunNoSlipNoFlipAlgo(float* F_b_out, float F_F_max,int p_i_max,int p_i, float mu_s,float* d_C1_COM,float M,float theta,float* d_C1_C2,float SB1,float R,float r,float I_A2,float* d_C2_COM,float SB2,float I_A1,float* d_A1_COM){
  // Step 1:
  float F_F_desired = DesiredGroundFriction(F_F_max, p_i_max, p_i);
  // Step 2: 
  float F_F2_max = MaximumGroundFriction(mu_s,d_C1_COM,M,theta,d_C1_C2);
  // Step 3: 
  float leftover1 = F_F_desired - (F_F2_max - SB1);
  //float F_b_out[2] = {0,0};
  //float* F_b_out = malloc(sizeof(float)*2);
  float F_F2_provided, F_b2_provided;
  if(leftover1 <= 0){
    // Step 4:
    F_F2_provided = F_F_desired;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b_out[0] = 0;
    F_b_out[1] = F_b2_provided;
  }
  else{
    // Step 5:
    F_F2_provided = F_F2_max - SB1;
    F_b2_provided = ProvidedDiskBraking(M, R, r, I_A2, F_F_desired, F_F2_provided);
    F_b_out[1] = F_b2_provided;
    
    // Step 6:
    // See value of d_A1_COM in main, it is purposely adjusted forwards
    float F_F1_NO = MaximumGroundFrictionNoseOver(M, theta, d_A1_COM);
    
    // Step 7:
    // d_C2_COM(1) = d_C2_COM(1) - 0.1; % Adjust COM backwards
    float F_F1_Smax = MaximumGroundFriction(mu_s,d_C2_COM,M,theta,d_C1_C2);

    // Step 8:
    float F_F1_max = min(F_F1_NO,F_F1_Smax);
    float leftover2 = leftover1 - (F_F1_max - SB2);

    float F_F1_provided, F_b1_provided;
    if(leftover2 <= 0){
        // Step 9:
        F_F1_provided = leftover1;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b_out[0] = F_b1_provided;
    }
    else{
        // Step 10:
        F_F1_provided = F_F1_max - SB2;
        F_b1_provided = ProvidedDiskBraking(M, R, r, I_A1, F_F_desired, F_F1_provided);
        F_b_out[0] = F_b1_provided;
    }
    
    Serial.println(F_b_out[0]);
    Serial.println(F_b_out[1]);
  }
  
  //return F_b_out;
}

void ForceToPWM(float* PWM, float* F_b_out){
  //float PWM[2] = {20, 80};
  //return PWM;
//  float calibration1 = 1;
//  float calibration2 = 1;
//  F_b_out[0] = F_b_out[0]*calibration1;
//  F_b_out[1] = F_b_out[1]*calibration2;
//  return F_b_out;

  //float* PWM = malloc(sizeof(float)*2);
  PWM[0] = 20;
  PWM[1] = 80;
  //return PWM
}

void MoveMotors(float* PWM){
  //Serial.println("success");
}

void setup() {}

void loop() {
  // READ INPUTS
  // Read the input potentiometer position
  int p_i = ReadPot();
  // Read the gyroscope angle
  float theta = ReadGyro();

  // Preventative System
  float* F_b_out = (float*)malloc(sizeof(float)*2);
  float* PWM = (float*)malloc(sizeof(float)*2);
  // F_b_out[0] = F_b1_out, F_b_out[1] = F_b2_out
  RunNoSlipNoFlipAlgo(F_b_out, F_F_max,p_i_max,p_i,mu_s,d_C1_COM,M,theta,d_C1_C2,SB1,R,r,I_A2,d_C2_COM,SB2,I_A1,d_A1_COM); 
  // PWM[0] = PWM_1, PWM[1] = PWM_2
  ForceToPWM(PWM, F_b_out);
  MoveMotors(PWM);
  
  free(F_b_out);
  free(PWM);
  // Reactive System
  // infinite loop that only breaks with a significant change in potentiometer position
}
