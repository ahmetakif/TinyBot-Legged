//This is the Arduino Code for the Arduino Nano connected the servo motors and MPU6050
//on the TinyBot Legged Robot
//Authored by Ahmet Akif Kaya | Aug/Sep 2020 
//v.14

//MPU6050 Libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU6050 Init
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

float IMUoffsetX = -1600.0;
float IMUoffsetY = -830.0;
const int divideBYX = 375; // Divide the original values to get an angle
const int divideBYY = 265; // Divide the original values to get an angle

//Servos
#include <Servo.h>
Servo fl1;
Servo fl2;
Servo fl3;
Servo fr1;
Servo fr2;
Servo fr3;
Servo bl1;
Servo bl2;
Servo bl3;
Servo br1;
Servo br2;
Servo br3;

//Servo Offsets
const int fl1o = 53;
const int fl2o = 80;
const int fl3o = 96;
const int fr1o = 108;
const int fr2o = 83;
const int fr3o = 94;
const int bl1o = 126;
const int bl2o = 115;
const int bl3o = 83;
const int br1o = 60;
const int br2o = 95;
const int br3o = 94;

//Servo Limit Angles - Limited Mechanically
const int fl1ul = fl1o + 50;  // offset plus  50 limit
const int fl1dl = fl1o - 35;  // offset minus 35 limit
const int fl2ul = fl2o + 38;  // offset plus  38 limit  // INVERSED MOTOR DIRECTION!
const int fl2dl = fl2o - 50;  // offset minus 50 limit  // INVERSED MOTOR DIRECTION!
const int fl3ul = fl3o + 55;  // offset plus  55 limit  // INVERSED MOTOR DIRECTION!
const int fl3dl = fl3o - 75;  // offset minus 75 limit  // INVERSED MOTOR DIRECTION!
//-----------------------------------------
const int fr1ul = fr1o + 35;  // offset plus  35 limit  // INVERSED MOTOR DIRECTION!
const int fr1dl = fr1o - 50;  // offset minus 50 limit  // INVERSED MOTOR DIRECTION!
const int fr2ul = fr2o + 50;  // offset plus  50 limit
const int fr2dl = fr2o - 38;  // offset minus 38 limit
const int fr3ul = fr3o + 75;  // offset plus  75 limit
const int fr3dl = fr3o - 55;  // offset minus 55 limit
//-----------------------------------------
const int bl1ul = bl1o + 35;  // offset plus  35 limit  // INVERSED MOTOR DIRECTION!
const int bl1dl = bl1o - 50;  // offset minus 50 limit  // INVERSED MOTOR DIRECTION!
const int bl2ul = bl2o + 48;  // offset plus  48 limit  // INVERSED MOTOR DIRECTION!
const int bl2dl = bl2o - 50;  // offset minus 50 limit  // INVERSED MOTOR DIRECTION!
const int bl3ul = bl3o + 58;  // offset plus  58 limit  // INVERSED MOTOR DIRECTION!
const int bl3dl = bl3o - 85;  // offset minus 85 limit  // INVERSED MOTOR DIRECTION!
//-----------------------------------------
const int br1ul = br1o + 50;  // offset plus  50 limit
const int br1dl = br1o - 35;  // offset minus 35 limit
const int br2ul = br2o + 50;  // offset plus  50 limit
const int br2dl = br2o - 48;  // offset minus 48 limit
const int br3ul = br3o + 85;  // offset plus  85 limit
const int br3dl = br3o - 58;  // offset minus 58 limit
//-----------------------------------------

// Servo Position Values without the Offsets
int fl1p=0,fl2p=0,fl3p=0,fr1p=0,fr2p=0,fr3p=0,bl1p=0,bl2p=0,bl3p=0,br1p=0,br2p=0,br3p=0; 

//Robot Properties
//---------------- General and for H(Z) axis / up and down movement
const int legfootlen = 45; //In milimeters // Both for leg and foot
const float standardH = 63.63; // Height of the robot in default position
float alphaA = 45.0; //Alpha angle in degrees as default position
float betaA = 90.0; //Beta angle in degrees as default position
float alphaR = (alphaA*PI)/180; // Conversion from degrees to radians
float betaR = (betaA*PI)/180;
//---------------- For X axis / Back and forth movement
float thetaA = 0;
float thetaR = (thetaA*PI)/180;
float h2 = 63.63;
//---------------- For Y axis / Right and left movement
float legYoffset = 20; //This is in milimeters // Will change for different robot designs
float phiR = atan(legYoffset/standardH);
float phiA = (phiR*180)/PI;
float tR = atan(standardH/legYoffset);
float tA = (tR*180)/PI;
float tphiA = (phiA + tA - 90);
float h3 = 63.63;
//---------------- Final alphaA angle, betaA angle, and tphiA angle calculations and conversions
int finalalphaA = (int)(-(alphaA-45));
int finalbetaA = (int)(2*betaA-90);
int finaltphiA = (int)(tphiA);

//////////INVERSE KINEMATICS/////////////////////////////////////////////
//Location Coordinate Properties for Each Leg - In Milimeters
float flH = 63.63;  //Robot up and down (Or Z axis, whatever you call it :))
float flX = 0;      //Robot back and forth
float flY = 0;      //Robot right and left
//----------------------------------------
float frH = 63.63;  //Robot up and down
float frX = 0;      //Robot back and forth
float frY = 0;      //Robot right and left
//----------------------------------------
float blH = 63.63;  //Robot up and down
float blX = 0;      //Robot back and forth
float blY = 0;      //Robot right and left
//----------------------------------------
float brH = 63.63;  //Robot up and down
float brX = 0;      //Robot back and forth
float brY = 0;      //Robot right and left
//----------------------------------------

//Location Coordinate Properties for Each Leg - In Milimeters ////// ROTATIONAL
float rflH = 63.63;  //Robot up and down (Or Z axis, whatever you call it :))
float rflX = 0;      //Robot back and forth
float rflY = 0;      //Robot right and left
//----------------------------------------
float rfrH = 63.63;  //Robot up and down
float rfrX = 0;      //Robot back and forth
float rfrY = 0;      //Robot right and left
//----------------------------------------
float rblH = 63.63;  //Robot up and down
float rblX = 0;      //Robot back and forth
float rblY = 0;      //Robot right and left
//----------------------------------------
float rbrH = 63.63;  //Robot up and down
float rbrX = 0;      //Robot back and forth
float rbrY = 0;      //Robot right and left
//----------------------------------------

//Location Coordinate Properties for Each Leg - In Milimeters ////// ROTATIONAL YAW
float ryflH = 63.63;  //Robot up and down (Or Z axis, whatever you call it :))
float ryflX = 0;      //Robot back and forth
float ryflY = 0;      //Robot right and left
//----------------------------------------
float ryfrH = 63.63;  //Robot up and down
float ryfrX = 0;      //Robot back and forth
float ryfrY = 0;      //Robot right and left
//----------------------------------------
float ryblH = 63.63;  //Robot up and down
float ryblX = 0;      //Robot back and forth
float ryblY = 0;      //Robot right and left
//----------------------------------------
float rybrH = 63.63;  //Robot up and down
float rybrX = 0;      //Robot back and forth
float rybrY = 0;      //Robot right and left
//----------------------------------------

//Location Coordinate Properties for Each Leg - In Milimeters ////// ROTATIONAL PITCH
float rpflH = 63.63;  //Robot up and down (Or Z axis, whatever you call it :))
float rpflX = 0;      //Robot back and forth
float rpflY = 0;      //Robot right and left
//----------------------------------------
float rpfrH = 63.63;  //Robot up and down
float rpfrX = 0;      //Robot back and forth
float rpfrY = 0;      //Robot right and left
//----------------------------------------
float rpblH = 63.63;  //Robot up and down
float rpblX = 0;      //Robot back and forth
float rpblY = 0;      //Robot right and left
//----------------------------------------
float rpbrH = 63.63;  //Robot up and down
float rpbrX = 0;      //Robot back and forth
float rpbrY = 0;      //Robot right and left
//----------------------------------------

//////////ROBOT ROTATION/////////////////////////////////////////////
//Robot Properties
const float rx = 42.5;
const float ry = 41;
const float rhip = sqrt(sq(rx)+sq(ry));
//----------------
float pA = 0;
float rA = 0;
float yA = 0;
float pR = (pA*PI)/180; 
float rR = (rA*PI)/180; 
float yR = (yA*PI)/180; 
//----------------
float tiR = atan(rx/standardH);
float tiA = (tiR*180)/PI; // tiA means the tip angle of the leg
float tiA2 = 90 - tiA;
float fA = 90 - (pA/2) - tiA;
float fR = (fA*PI)/180; 
float fA2 = 180 - 2*tiA - fA;
float fR2 = (fA2*PI)/180;
//----------------
float frontH = standardH;
float rearH = standardH;
//----------------

void setup() {
  //Normal setup
  Serial.begin(9600);
  Serial.setTimeout(5);
  //Welcome message to the raspberry 
  //(Essentially all the messages are sent to the python script on raspberry pi of TinyBot Legged)
  Serial.println("Hello from the arduino nano on the TinyBot Legged.");
  //Attach the servos
  fl1.attach(8);
  fl2.attach(11);
  fl3.attach(10);
  fr1.attach(4);
  fr2.attach(2);
  fr3.attach(7);
  bl1.attach(12);
  bl2.attach(9);
  bl3.attach(5);
  br1.attach(3);
  br2.attach(6);
  br3.attach(13);
  //Moving Servos to offset
  fl1.write(fl1o);
  fl2.write(fl2o);
  fl3.write(fl3o);
  fr1.write(fr1o);
  fr2.write(fr2o);
  fr3.write(fr3o);
  bl1.write(bl1o);
  bl2.write(bl2o);
  bl3.write(bl3o);
  br1.write(br1o);
  br2.write(br2o);
  br3.write(br3o);
  //MPOU6050 Setup
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil("\n");
    String command = data.substring(0,1);
    int value = 0;
    int leg = 5; //FL:1, FR:2, BL:3, BR:4, ALL:5
    if (command == "_" or command == "=" or command == "&" or command == "?" or command == "!" or command == ";"){
      String legstr = data.substring(1,2);
      String valuestr = data.substring(2);
      leg = legstr.toInt();
      value = valuestr.toInt();
    }
    //----------------------
    //----------------------
    //----------------------
    //----------------------
    if (command == "_"){
      // Elevate axis
      if (leg == 1){
        flH = -value;
      }
      else if (leg == 2){
        frH = -value;
      }
      else if (leg == 3){
        blH = -value;        
      }
      else if (leg == 4){
        brH = -value;
      }
      else{
        flH = value;
        frH = value;
        blH = value;
        brH = value;
      }
      Serial.println((frontH+rearH)/2);
      kinematics("fl",flH+standardH,flX,flY,0);
      kinematics("fr",frH+standardH,frX,frY,0);
      kinematics("bl",blH+standardH,blX,blY,0);
      kinematics("br",brH+standardH,brX,brY,0);
    }
    else if (command == "="){
      //Back and forth axis
      if (leg == 1){
        flX = -value;
      }
      else if (leg == 2){
        frX = -value;
      }
      else if (leg == 3){
        blX = -value;        
      }
      else if (leg == 4){
        brX = -value;
      }
      else{
        flX = value;
        frX = value;
        blX = value;
        brX = value;
      }
      Serial.println(value);
      kinematics("fl",flH+standardH,flX,flY,0);
      kinematics("fr",frH+standardH,frX,frY,0);
      kinematics("bl",blH+standardH,blX,blY,0);
      kinematics("br",brH+standardH,brX,brY,0);
    }
    else if (command == "&"){
      // Right and left axis
      if (leg == 1){
        flY = -value;
      }
      else if (leg == 2){
        frY = value;
      }
      else if (leg == 3){
        blY = -value;        
      }
      else if (leg == 4){
        brY = value;
      }
      else{
        flY = value;
        frY = -value;
        blY = value;
        brY = -value;
      }
      Serial.println(value);
      kinematics("fl",flH+standardH,flX,flY,0);
      kinematics("fr",frH+standardH,frX,frY,0);
      kinematics("bl",blH+standardH,blX,blY,0);
      kinematics("br",brH+standardH,brX,brY,0);
    }
    else if (command == "?"){
      // Pitch axis
      pA = value;
      Serial.println(pA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    else if (command == "!"){
      // Roll axis
      rA = value;
      Serial.println(rA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    else if (command == ";"){
      // Yaw axis
      yA = value;
      Serial.println(yA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    //----------------------
    //----------------------
    //----------------------
    //----------------------
    else if (command == "2") {
      fl1p = fl1p + 1;
      Serial.println(fl1p);
    }
    else if (command == "w") {
      fl1p = fl1p - 1;
      Serial.println(fl1p);
    }
    else if (command == "q") {
      fl2p = fl2p + 1;
      Serial.println(fl2p);
    }
    else if (command == "e") {
      fl2p = fl2p - 1;
      Serial.println(fl2p);
    }
    else if (command == "1") {
      fl3p = fl3p + 1;
      Serial.println(fl3p);
    }
    else if (command == "3") {
      fl3p = fl3p - 1;
      Serial.println(fl3p);
    }
    //----------------------
    else if (command == "5") {
      fr1p = fr1p + 1;
      Serial.println(fr1p);
    }
    else if (command == "t") {
      fr1p = fr1p - 1;
      Serial.println(fr1p);
    }
    else if (command == "r") {
      fr2p = fr2p + 1;
      Serial.println(fr2p);
    }
    else if (command == "y") {
      fr2p = fr2p - 1;
      Serial.println(fr2p);
    }
    else if (command == "4") {
      fr3p = fr3p + 1;
      Serial.println(fr3p);
    }
    else if (command == "6") {
      fr3p = fr3p - 1;
      Serial.println(fr3p);
    }
    //----------------------
    else if (command == "s") {
      bl1p = bl1p + 1;
      Serial.println(bl1p);
    }
    else if (command == "x") {
      bl1p = bl1p - 1;
      Serial.println(bl1p);
    }
    else if (command == "z") {
      bl2p = bl2p + 1;
      Serial.println(bl2p);
    }
    else if (command == "c") {
      bl2p = bl2p - 1;
      Serial.println(bl2p);
    }
    else if (command == "a") {
      bl3p = bl3p + 1;
      Serial.println(bl3p);
    }
    else if (command == "d") {
      bl3p = bl3p - 1;
      Serial.println(bl3p);
    }
    //----------------------
    else if (command == "g") {
      br1p = br1p + 1;
      Serial.println(br1p);
    }
    else if (command == "b") {
      br1p = br1p - 1;
      Serial.println(br1p);
    }
    else if (command == "v") {
      br2p = br2p + 1;
      Serial.println(br2p);
    }
    else if (command == "n") {
      br2p = br2p - 1;
      Serial.println(br2p);
    }
    else if (command == "f") {
      br3p = br3p + 1;
      Serial.println(br3p);
    }
    else if (command == "h") {
      br3p = br3p - 1;
      Serial.println(br3p);
    }
    //----------------------
    //----------------------
    else if (command == "7") {
      resetdefpos();
    }
    //---------------------- TRANSLATION
    //----------------------
    else if (command == "+"){
      flH = flH + 1;
      frH = frH + 1;
      blH = blH + 1;
      brH = brH + 1;
      Serial.println((frontH+rearH)/2);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    else if (command == "-"){
      flH = flH - 1;
      frH = frH - 1;
      blH = blH - 1;
      brH = brH - 1;
      Serial.println((frontH+rearH)/2);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    //----------------------  
    else if (command == "*"){
      flX = flX + 1;
      frX = frX + 1;
      blX = blX + 1;
      brX = brX + 1;
      Serial.println(flX);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    else if (command == "/"){
      flX = flX - 1;
      frX = frX - 1;
      blX = blX - 1;
      brX = brX - 1;
      Serial.println(flX);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    //----------------------
    else if (command == "9"){
      flY = flY + 1;
      frY = frY - 1;
      blY = blY + 1;
      brY = brY - 1;
      Serial.println(flY);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    else if (command == "8"){
      flY = flY - 1;
      frY = frY + 1;
      blY = blY - 1;
      brY = brY + 1;
      Serial.println(flY);
      kinematics("fl",flH,flX,flY,0);
      kinematics("fr",frH,frX,frY,0);
      kinematics("bl",blH,blX,blY,0);
      kinematics("br",brH,brX,brY,0);
    }
    //---------------------- ROTATION
    //----------------------
    else if (command == "j"){
      pA = pA + 1;
      Serial.println(pA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    else if (command == "k"){
      pA = pA - 1;
      Serial.println(pA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    //----------------------
    else if (command == "o"){
      rA = rA + 1;
      Serial.println(rA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    else if (command == "p"){
      rA = rA - 1;
      Serial.println(rA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    //----------------------
    else if (command == "."){
      yA = yA + 1;
      Serial.println(yA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    else if (command == ","){
      yA = yA - 1;
      Serial.println(yA);
      robotrotation((frontH+rearH)/2,pA,rA,yA);
    }
    //----------------------
    //----------------------
    else if (command == "<"){
      //IMU data is requested for READ
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print("a/g:\t");
      Serial.print((ax + IMUoffsetX)/divideBYX); Serial.print("\t");
      Serial.print((ay + IMUoffsetY)/divideBYY); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
    }
    else if (command == ">"){
      //IMU data is requested for AUTO
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.println(String((ax + IMUoffsetX)/divideBYX) + ":" + String((ay + IMUoffsetY)/divideBYY));
    }
  }
  moveservo("fl1");
  moveservo("fl2");
  moveservo("fl3");
  moveservo("fr1");
  moveservo("fr2");
  moveservo("fr3");
  moveservo("bl1");
  moveservo("bl2");
  moveservo("bl3");
  moveservo("br1");
  moveservo("br2");
  moveservo("br3");
}

void robotrotation(float H, float pitch, float roll, float yaw){
  // For yaw axis
  yA = yaw;
  //GENERAL
  yR = (abs(yA)*PI)/180; 
  float dif = 2*rhip*sin(yR/2);
  if (yA >= 0){
    //FL
    tiR = atan(ry/rx);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (yA/2) - tiA2;
    fR = (fA*PI)/180;
    float difX = (dif*sin(fR))*(+1);
    float difY = (dif*cos(fR))*(-1);
    float h5 = sqrt(sq(H)+sq(difX));
    float difH = sqrt(sq(h5)+sq(difY)) - standardH;
    ryflH = rflH + difH;
    ryflX = rflX + difX;
    ryflY = rflY + difY;
    //FR
    tiR = atan(rx/ry);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (yA/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*cos(fR))*(-1);
    difY = (dif*sin(fR))*(+1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    ryfrH = rfrH + difH;
    ryfrX = rfrX + difX;
    ryfrY = rfrY + difY;
    //BL
    tiR = atan(rx/ry);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (yA/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*cos(fR))*(+1);
    difY = (dif*sin(fR))*(+1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    ryblH = rblH + difH;
    ryblX = rblX + difX;
    ryblY = rblY + difY;
    //BR
    tiR = atan(ry/rx);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (yA/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*sin(fR))*(-1);
    difY = (dif*cos(fR))*(-1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    rybrH = rbrH + difH;
    rybrX = rbrX + difX;
    rybrY = rbrY + difY;
  }
  else{
    //FL
    tiR = atan(rx/ry);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(yA)/2) - tiA2;
    fR = (fA*PI)/180;
    float difX = (dif*cos(fR))*(-1);
    float difY = (dif*sin(fR))*(+1);
    float h5 = sqrt(sq(H)+sq(difX));
    float difH = sqrt(sq(h5)+sq(difY))  - standardH;
    ryflH = rflH + difH;
    ryflX = rflX + difX;
    ryflY = rflY + difY;
    //FR
    tiR = atan(ry/rx);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(yA)/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*sin(fR))*(+1);
    difY = (dif*cos(fR))*(-1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    ryfrH = rfrH + difH;
    ryfrX = rfrX + difX;
    ryfrY = rfrY + difY;
    //BL
    tiR = atan(ry/rx);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(yA)/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*sin(fR))*(-1);
    difY = (dif*cos(fR))*(-1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    ryblH = rblH + difH;
    ryblX = rblX + difX;
    ryblY = rblY + difY;
    //BR
    tiR = atan(rx/ry);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(yA)/2) - tiA2;
    fR = (fA*PI)/180;
    difX = (dif*cos(fR))*(+1);
    difY = (dif*sin(fR))*(+1);
    h5 = sqrt(sq(H)+sq(difX));
    difH = sqrt(sq(h5)+sq(difY))  - standardH;
    rybrH = rbrH + difH;
    rybrX = rbrX + difX;
    rybrY = rbrY + difY;
  }

  // For pitch axis
  pA = pitch;
  // PASSING PARAMETERS
  rpflY = ryflY;
  rpfrY = ryfrY;
  rpblY = ryblY;
  rpbrY = rybrY;
  if (pA >= 0){
    //Rear Leg
    float hip2 = sqrt(sq(H)+sq(rx));
    tiR = atan(rx/H);
    tiA = (tiR*180)/PI; // tiA means the tip angle of the leg
    fA = 90 - (pA/2) - tiA;
    fR = (fA*PI)/180;
    pR = (pA*PI)/180; 
    float dif = 2*hip2*sin(pR/2);
    float difH = (dif*cos(fR))*(-1);
    float difX = (dif*sin(fR))*(-1);
    rpblH = ryblH + difH, rpbrH = rybrH + difH;
    rpblX = ryblX + difX, rpbrX = rybrX + difX;
    //Front Leg
    hip2 = sqrt(sq(H)+sq(rx));
    tiR = atan(rx/H);
    tiA = (tiR*180)/PI; // tiA means the tip angle of the leg
    fA = 90 - (pA/2) - tiA;
    fA2 = 180 - 2*tiA - fA;
    fR = (fA*PI)/180;
    fR2 = (fA2*PI)/180;
    pR = (pA*PI)/180; 
    dif = 2*hip2*sin(pR/2);
    difH = (dif*cos(fR2))*(+1);
    difX = (dif*sin(fR2))*(-1);
    rpflH = ryflH + difH, rpfrH = ryfrH + difH;
    rpflX = ryflX + difX, rpfrX = ryfrX + difX;
  }
  else{
    //Front Leg
    float hip2 = sqrt(sq(H)+sq(rx));
    tiR = atan(rx/H);
    tiA = (tiR*180)/PI; // tiA means the tip angle of the leg
    fA = 90 - (abs(pA)/2) - tiA;
    fR = (fA*PI)/180;
    pR = (abs(pA)*PI)/180; 
    float dif = 2*hip2*sin(pR/2);
    float difH = (dif*cos(fR))*(-1);
    float difX = (dif*sin(fR))*(+1);
    rpflH = ryflH + difH, rpfrH = ryfrH + difH;
    rpflX = ryflX + difX, rpfrX = ryfrX + difX;
    //Rear Leg
    hip2 = sqrt(sq(H)+sq(rx));
    tiR = atan(rx/H);
    tiA = (tiR*180)/PI; // tiA means the tip angle of the leg
    fA = 90 - (abs(pA)/2) - tiA;
    fA2 = 180 - 2*tiA - fA;
    fR = (fA*PI)/180;
    fR2 = (fA2*PI)/180;
    pR = (abs(pA)*PI)/180; 
    dif = 2*hip2*sin(pR/2);
    difH = (dif*cos(fR2))*(+1);
    difX = (dif*sin(fR2))*(+1);
    rpblH = ryblH + difH, rpbrH = rybrH + difH;
    rpblX = ryblX + difX, rpbrX = rybrX + difX;
  }

  // For roll axis
  rA = roll;
  // PASSING PARAMETERS
  flX = rpflX;
  frX = rpfrX;
  blX = rpblX;
  brX = rpbrX;
  if (rA >= 0){
    //---------------------
    //For the rear two legs
    //Rear left leg
    float hip2 = sqrt(sq(rearH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/rearH);
    tiA = (tiR*180)/PI;
    fA = 90 - (rA/2) - tiA;
    fR = (fA*PI)/180;
    rR = (rA*PI)/180; 
    float dif = 2*hip2*sin(rR/2);
    float difH = (dif*cos(fR))*(-1);
    float difY = (dif*sin(fR))*(+1);
    blH = rpblH + difH;
    blY = rpblY + difY;
    //Rear right leg
    hip2 = sqrt(sq(rearH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/rearH);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (rA/2) - tiA2;
    fR = (fA*PI)/180;
    rR = (rA*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*sin(fR))*(+1);
    difY = (dif*cos(fR))*(-1);
    brH = rpbrH + difH;
    brY = rpbrY + difY;
    //---------------------
    //For the front two legs
    //Front left leg
    hip2 = sqrt(sq(frontH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/frontH);
    tiA = (tiR*180)/PI;
    fA = 90 - (rA/2) - tiA;
    fR = (fA*PI)/180;
    rR = (rA*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*cos(fR))*(-1);
    difY = (dif*sin(fR))*(+1);
    flH = rpflH + difH;
    flY = rpflY + difY;
    //Front right leg
    hip2 = sqrt(sq(frontH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/frontH);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (rA/2) - tiA2;
    fR = (fA*PI)/180;
    rR = (rA*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*sin(fR))*(+1);
    difY = (dif*cos(fR))*(-1);
    frH = rpfrH + difH;
    frY = rpfrY + difY;
    //---------------------
  }
  else{
    //---------------------
    //For the rear two legs
    //Rear right leg
    float hip2 = sqrt(sq(rearH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/rearH);
    tiA = (tiR*180)/PI;
    fA = 90 - (abs(rA)/2) - tiA;
    fR = (fA*PI)/180;
    rR = (abs(rA)*PI)/180; 
    float dif = 2*hip2*sin(rR/2);
    float difH = (dif*cos(fR))*(-1);
    float difY = (dif*sin(fR))*(+1);
    brH = rpbrH + difH;
    brY = rpbrY + difY;
    //Rear left leg
    hip2 = sqrt(sq(rearH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/rearH);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(rA)/2) - tiA2;
    fR = (fA*PI)/180;
    rR = (abs(rA)*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*sin(fR))*(+1);
    difY = (dif*cos(fR))*(-1);
    blH = rpblH + difH;
    blY = rpblY + difY;
    //---------------------
    //For the front two legs
    //Front right leg
    hip2 = sqrt(sq(frontH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/frontH);
    tiA = (tiR*180)/PI;
    fA = 90 - (abs(rA)/2) - tiA;
    fR = (fA*PI)/180;
    rR = (abs(rA)*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*cos(fR))*(-1);
    difY = (dif*sin(fR))*(+1);
    frH = rpfrH + difH;
    frY = rpfrY + difY;
    //Front left leg
    hip2 = sqrt(sq(frontH)+sq(ry + legYoffset));
    tiR = atan((ry + legYoffset)/frontH);
    tiA = (tiR*180)/PI;
    tiA2 = 90 - tiA;
    fA = 90 - (abs(rA)/2) - tiA2;
    fR = (fA*PI)/180;
    rR = (abs(rA)*PI)/180; 
    dif = 2*hip2*sin(rR/2);
    difH = (dif*sin(fR))*(+1);
    difY = (dif*cos(fR))*(-1);
    flH = rpflH + difH;
    flY = rpflY + difY;
    //---------------------
  }
  // Execution on the kinematic model
  kinematics("fl",flH,flX,flY,1);
  kinematics("fr",frH,frX,frY,1);
  kinematics("bl",blH,blX,blY,1);
  kinematics("br",brH,brX,brY,1);
}

void kinematics(String leg, float h, float x, float y, boolean isrot){
  //X axis - Backandforth
  if (x >= 0){ // If the x is positive means the leg wants to move forward
    thetaR = -(atan(x/h));
    thetaA = (thetaR*180)/PI;
    h2 = sqrt(sq(h)+sq(x));
  }
  else{        // If the x is negative means the leg wants to move backward
    thetaR = atan(abs(x)/h);
    thetaA = (thetaR*180)/PI;
    h2 = sqrt(sq(h)+sq(x));
  }

  //Y axis - Rightandleft
  if (y >= 0){
    phiR = atan((legYoffset + y)/h2);
    phiA = (phiR*180)/PI;
    float hip = sqrt(sq(h2)+sq(legYoffset+y));
    h3 = sqrt(sq(hip)-sq(legYoffset));
    tR = acos(legYoffset/hip);
    tA = (tR*180)/PI;
    tphiA = (phiA + tA - 90);
  }
  else if (y < 0 and y >= -legYoffset){
    phiR = atan((legYoffset - abs(y))/h2);
    phiA = (phiR*180)/PI;
    float hip = sqrt(sq(h2)+sq(legYoffset - abs(y)));
    h3 = sqrt(sq(hip)-sq(legYoffset));
    tR = acos(legYoffset/hip);
    tA = (tR*180)/PI;
    tphiA = -(90 - phiA - tA);
  }
  else{
    phiR = atan((abs(y) - legYoffset)/h2);
    phiA = (phiR*180)/PI;
    float hip = sqrt(sq(h2)+sq(abs(y)-legYoffset));
    h3 = sqrt(sq(hip)-sq(legYoffset));
    tR = acos(legYoffset/hip);
    tA = (tR*180)/PI;
    tphiA = -(90 + phiA - tA);
  }

  //H axis - Elevate (Or Z axis, whatever you call it :))
  alphaR = acos(h3/(2*legfootlen));
  alphaA = (alphaR*180)/PI;
  betaR = asin(h3/(2*legfootlen));
  betaA = (betaR*180)/PI;

  //Final alphaA angle, betaA angle, and tphiA angle calculations and conversions
  alphaA = alphaA + thetaA;
  finalalphaA = (int)(-(alphaA-45));
  finalbetaA = (int)(2*betaA-90);
  finaltphiA = (int)(tphiA);

  //Which Leg
  if (leg == "fl"){
    fl1p = finaltphiA;
    fl2p = finalalphaA;
    fl3p = finalbetaA;
    if (isrot == 0){
      rflH = h;
      rflX = x;
      rflY = y;
      frontH = h;
    }
  }
  else if (leg == "fr"){
    fr1p = finaltphiA;
    fr2p = finalalphaA;
    fr3p = finalbetaA;
    if (isrot == 0){
      rfrH = h;
      rfrX = x;
      rfrY = y;
      frontH = h;
    }
  }
  else if (leg == "bl"){
    bl1p = finaltphiA;
    bl2p = finalalphaA;
    bl3p = finalbetaA;
    if (isrot == 0){
      rblH = h;
      rblX = x;
      rblY = y;
      rearH = h;
    }
  }
  else if (leg == "br"){
    br1p = finaltphiA;
    br2p = finalalphaA;
    br3p = finalbetaA;
    if (isrot == 0){
      rbrH = h;
      rbrX = x;
      rbrY = y;
      rearH = h;
    }
  }
}

void resetdefpos(){
  // Sets all the servo position variables to their defaults.
  fl1p=0,fl2p=0,fl3p=0,fr1p=0,fr2p=0,fr3p=0,bl1p=0,bl2p=0,bl3p=0,br1p=0,br2p=0,br3p=0;
  //---------------------
  flX=0,frX=0,blX=0,brX=0,flY=0,frY=0,blY=0,brY=0;
  flH=63.63,frH=63.63,blH=63.63,brH=63.63;
  //---------------------
  rflX=0,rfrX=0,rblX=0,rbrX=0,rflY=0,rfrY=0,rblY=0,rbrY=0;
  rflH=63.63,rfrH=63.63,rblH=63.63,rbrH=63.63;
  //---------------------
  rpflX=0,rpfrX=0,rpblX=0,rpbrX=0,rpflY=0,rpfrY=0,rpblY=0,rpbrY=0;
  rpflH=63.63,rpfrH=63.63,rpblH=63.63,rpbrH=63.63;
  //---------------------
  ryflX=0,ryfrX=0,ryblX=0,rybrX=0,ryflY=0,ryfrY=0,ryblY=0,rybrY=0;
  ryflH=63.63,ryfrH=63.63,ryblH=63.63,rybrH=63.63;
  //---------------------
  pA=0,rA=0,yA=0;
  h2=63.63,h3=63.63;
  //---------------------
  rearH = 63.63;
  frontH = 63.63;
  //---------------------
}

void moveservo(String whichservo){
  if (whichservo == "fl1"){
    if (fl1o + fl1p < fl1ul and fl1o + fl1p > fl1dl){
      fl1.write(fl1o + fl1p);
    }
    else{
      if (fl1o + fl1p > fl1ul){
        fl1p = fl1ul - fl1o;
      }
      else if (fl1o + fl1p < fl1dl){
        fl1p = fl1dl - fl1o;
      }
    }
  }
  else if (whichservo == "fl2"){
    if (fl2o - fl2p < fl2ul and fl2o - fl2p > fl2dl){
      fl2.write(fl2o - fl2p);
    }
    else{
      if (fl2o - fl2p > fl2ul){
        fl2p = fl2o - fl2ul;
      }
      else if (fl2o - fl2p < fl2dl){
        fl2p = fl2o - fl2dl;
      }
    }
  }
  else if (whichservo == "fl3"){
    if (fl3o - fl3p < fl3ul and fl3o - fl3p > fl3dl){
      fl3.write(fl3o - fl3p);
    }
    else{
      if (fl3o - fl3p > fl3ul){
        fl3p = fl3o - fl3ul;
      }
      else if (fl3o - fl3p < fl3dl){
        fl3p = fl3o - fl3dl;
      }
    }
  }
  else if (whichservo == "fr1"){
    if (fr1o - fr1p < fr1ul and fr1o - fr1p > fr1dl){
      fr1.write(fr1o - fr1p);
    }
    else{
      if (fr1o - fr1p > fr1ul){
        fr1p = fr1o - fr1ul;
      }
      else if (fr1o - fr1p < fr1dl){
        fr1p = fr1o - fr1dl;
      }
    }
  }
  else if (whichservo == "fr2"){
    if (fr2o + fr2p < fr2ul and fr2o + fr2p > fr2dl){
      fr2.write(fr2o + fr2p); 
    }
    else{
      if (fr2o + fr2p > fr2ul){
        fr2p = fr2ul - fr2o;
      }
      else if (fr2o + fr2p < fr2dl){
        fr2p = fr2dl - fr2o;
      }
    }
  }
  else if (whichservo == "fr3"){
    if (fr3o + fr3p < fr3ul and fr3o + fr3p > fr3dl){
      fr3.write(fr3o + fr3p); 
    }
    else{
      if (fr3o + fr3p > fr3ul){
        fr3p = fr3ul - fr3o;
      }
      else if (fr3o + fr3p < fr3dl){
        fr3p = fr3dl - fr3o;
      }
    }
  }
  else if (whichservo == "bl1"){
    if (bl1o - bl1p < bl1ul and bl1o - bl1p > bl1dl){
      bl1.write(bl1o - bl1p);
    }
    else{
      if (bl1o - bl1p > bl1ul){
        bl1p = bl1o - bl1ul;
      }
      else if (bl1o - bl1p < bl1dl){
        bl1p = bl1o - bl1dl;
      }
    }
  }
  else if (whichservo == "bl2"){
    if (bl2o - bl2p < bl2ul and bl2o - bl2p > bl2dl){
      bl2.write(bl2o - bl2p);
    }
    else{
      if (bl2o - bl2p > bl2ul){
        bl2p = bl2o - bl2ul;
      }
      else if (bl2o - bl2p < bl2dl){
        bl2p = bl2o - bl2dl;
      }
    }
  }
  else if (whichservo == "bl3"){
    if (bl3o - bl3p < bl3ul and bl3o - bl3p > bl3dl){
      bl3.write(bl3o - bl3p);
    }
    else{
      if (bl3o - bl3p > bl3ul){
        bl3p = bl3o - bl3ul;
      }
      else if (bl3o - bl3p < bl3dl){
        bl3p = bl3o - bl3dl;
      }
    }
  }
  else if (whichservo == "br1"){
    if (br1o + br1p < br1ul and br1o + br1p > br1dl){
      br1.write(br1o + br1p); 
    }
    else{
      if (br1o + br1p > br1ul){
        br1p = br1ul - br1o;
      }
      else if (br1o + br1p < br1dl){
        br1p = br1dl - br1o;
      }
    }
  }
  else if (whichservo == "br2"){
    if (br2o + br2p < br2ul and br2o + br2p > br2dl){
      br2.write(br2o + br2p); 
    }
    else{
      if (br2o + br2p > br2ul){
        br2p = br2ul - br2o;
      }
      else if (br2o + br2p < br2dl){
        br2p = br2dl - br2o;
      }
    }
  }
  else if (whichservo == "br3"){
    if (br3o + br3p < br3ul and br3o + br3p > br3dl){
      br3.write(br3o + br3p); 
    }
    else{
      if (br3o + br3p > br3ul){
        br3p = br3ul - br3o;
      }
      else if (br3o + br3p < br3dl){
        br3p = br3dl - br3o;
      }
    }
  }
}
