Senario 6:
 ...........(from Integrating PQR Into the World Frame lesson).............
1-figure out the standard deviation:
I have done the STD by python, you could find the folder name ....(Scenario 6 STD)

2- then I use the STD at the top of  config\06_SensorNoise
MeasuredStdDev_GPSPosXY = 0.6964075466374975
MeasuredStdDev_AccelXY = 0.47565951616778485

3- complete UpdateFromIM function:
R matrics                  ---> a
multiplay R by the gyro    ---> b
update Eurle_angles        ---> c
normalize yaw to -pi .. pi ---> d

  .........(Matrics R)..........	
  Mat3x3F R = Mat3x3F::Zeros(); ---> a
  R(0, 0) = 1; ---> a
  R(0, 1) = sin(rollEst)*tan(pitchEst); ---> a
  R(0, 2) = cos(rollEst)*tan(pitchEst); ---> a
  R(1, 0) = 0; ---> a
  R(1, 1) = cos(rollEst); ---> a
  R(1, 2) = -sin(rollEst); ---> a
  R(2, 0) = 0; ---> a
  R(2, 1) = sin(rollEst) / cos(pitchEst); ---> a
  R(2, 2) = cos(rollEst) / cos(pitchEst); ---> a

  ........(Multiplaying)........
  V3F Eurle_angles = R * gyro; ---> b

  .......................(Updating)........................
  float predictedPitch = pitchEst + dtIMU * Eurle_angles.y; ---> c
  float predictedRoll = rollEst + dtIMU * Eurle_angles.x; ---> c
  ekfState(6) = ekfState(6) + dtIMU * Eurle_angles.z; ---> c

  ..................(normalizing)...............  
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI; ---> d
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI; ---> d

  *(First correction)*   
  ((((((((((((((((From udacity reviw (report):
  Quaternion<float> quat = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  quat.IntegrateBodyRate(gyro, dtIMU);

  float predictedPitch = quat.Pitch();
  float predictedRoll = quat.Roll();
  ekfState(6) = quat.Yaw();
  ))))))))))))))))))))))))))))))))))))


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Senario 7:

I found from Dead_Reckoning_3D-Solution Last Checkpoint: 04/24/2018:

(((((((    def dead_reckoning_position(self, measured_acceleration, dt):
        self.X[0] = self.X[0] + self.X[5] * dt   # x coordianate x= x + \dot{x} * dt ---> a
        self.X[1] = self.X[1] + self.X[6] * dt   # y coordianate y= y + \dot{y} * dt ---> b
        self.X[2] = self.X[2] + self.X[7] * dt   # z coordianate z= z + \dot{z} * dt ---> c
        self.X[5] = self.X[5] + a[0] * dt        # change in velocity along the x is a_x * dt ---> d 
        self.X[6] = self.X[6] + a[1] * dt        # change in velocity along the y is a_y * dt ---> e
        self.X[7] = self.X[7] + a[2] * dt        # change in velocity along the z is a_z * dt ---> f
))))))))

		so I change it to c++:
 float x = curState(0);
 float y = curState(1);
 float z = curState(2);

 float x_dot = curState(4);
 float y_dot = curState(5);
 float z_dot = curState(6);

  V3F a = attitude.Rotate_BtoI(accel);
  float a_x = a.x;
  float a_y = a.y;
  float a_z = a.z;

  predictedState(0) = x + (dt * x_dot); ---> a
  predictedState(1) = y + (dt * y_dot); ---> b
  predictedState(2) = z + (dt * z_dot); ---> c
  predictedState(3) = x_dot + (dt * a_x); ---> d
  predictedState(4) = y_dot + (dt * a_y); ---> e
  predictedState(5) = z_dot + (dt * a_z) - (dt * 9.81); ---> f //I add from hints the minus gravity

<----- *(Second correction)* ----->
  (((((((((
  From udacity reviw (report):
  // integrate positions
  predictedState(0) += curState(3)*dt;
  predictedState(1) += curState(4)*dt;
  predictedState(2) += curState(5)*dt;

  // integrate velocity
  accelG = attitude.Rotate_BtoI(accel) - V3F(0, 0, 9.81f);
  predictedState(3) += accelG[0] * dt;
  predictedState(4) += accelG[1] * dt;
  predictedState(5) += accelG[2] * dt;
  ))))))))))))))))))))))))))))))))))))))

  \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
  Senario 8:
  I change the numbers by  multiplaying 0.1 from QuadEstimatorEKF.txt: 
  
QPosXYStd = .005 
QPosZStd = .005
QVelXYStd = .005
QVelZStd = .01
QYawStd = .2 // from 0.005 to 0.2
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Senario 9:

  From the PDF Estimation for Quadrotors part 7.2 (https://www.overleaf.com/read/vymfngphcccj): 
  equation (52) I translated to c++:
  R_bg_prime :
  RbgPrime(0, 0) = ( - cos(pitch) * sin(yaw) );
  RbgPrime(0, 1) = ( - sin(roll) * sin(pitch) * sin(yaw) ) - ( cos(roll) * cos(yaw) );
  RbgPrime(0, 2) = ( - cos(roll) * sin(pitch) * sin(yaw) ) + ( sin(roll) * cos(yaw) );
  RbgPrime(1, 0) = ( cos(pitch) * cos(yaw) );
  RbgPrime(1, 1) = ( sin(roll) * sin(pitch) * cos(yaw) ) - ( cos(roll) * sin(yaw) );
  RbgPrime(1, 2) = ( cos(roll) * sin(pitch) * cos(yaw) ) + ( sin(roll) * sin(yaw) );
  RbgPrime(2, 0) = 0;
  RbgPrime(2, 1) = 0;
  RbgPrime(2, 2) = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Senario 10:

  //  - Your current estimated yaw can be found in the state vector: ekfState(6) ---> a
  //  - Make sure to normalize the difference between your measured and estimated yaw ---> b
  //    (you don't want to update your yaw the long way around the circle)
  

  zFromX(0) = ekfState(6); ---> a
  float PI = 3.14; ---> b

  // just take the difference between my measured and estimated yaw
  float delta = magYaw - ekfState(6); ---> b 

  // I found this trick to normalize from http://www.cplusplus.com/forum/beginner/66073/ at the end of the page.
  if (delta > PI) {zFromX(0) = zFromX(0) + 2* PI;} ---> b
  if (delta < -PI) {zFromX(0) = zFromX(0) - 2* PI;} ---> b

  // I found hPrime from (Estimation for Quadrotors) PDF on Magnetometer 7.3.2 section hPrime = [0,0,0,0,0,0,1]:
  hPrime(0, 6) = 1;
  


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Senario 12:
 .....................(Section A).........................

 GPS update: By updating Update(z, hPrime, R_GPS, zFromX):
  z ---> a
  hPrime ---> b
  R_GPS ---> c
  zFromX ---> d

Udacity already wrote the z:
  .....(z).....
  z(0) = pos.x; ---> a
  z(1) = pos.y; ---> a
  z(2) = pos.z; ---> a
  z(3) = vel.x; ---> a
  z(4) = vel.y; ---> a
  z(5) = vel.z; ---> a

 From Estimation for Quadrotors PDF section 7.3.1 GPS https://www.overleaf.com/read/vymfngphcccj :
  ....(hPrime).....
  hPrime(0, 0) = 1; ---> b 
  hPrime(1, 1) = 1; ---> b 
  hPrime(2, 2) = 1; ---> b 
  hPrime(3, 3) = 1; ---> b 
  hPrime(4, 4) = 1; ---> b 
  hPrime(5, 5) = 1; ---> b 
   
    
from  //void Predict//  Udacity made  //VectorXf newState//  then Udacity wrote  //ekfState = newState// : 
which mean the newState now is saved on ekfState:
so   :
  .......(zFromX)........ 
  zFromX(0) = ekfState(0); ---> d
  zFromX(1) = ekfState(1); ---> d
  zFromX(2) = ekfState(2); ---> d
  zFromX(3) = ekfState(3); ---> d
  zFromX(4) = ekfState(4); ---> d
  zFromX(5) = ekfState(5); ---> d

  <----- *(Third correction)* ----->
(((((((((((((((((((((((((((From Udacity review (report):
  int diagSize = QUAD_EKF_NUM_STATES - 1;
  hPrime.topLeftCorner(diagSize, diagSize) = MatrixXf::Identity(diagSize, diagSize);
  zFromX = hPrime * ekfState;
)))))))))))))))))))))))))))

  But here we have to notice that  // Predict function// have to be completed to find zFromX:

.......................................(Section B)........................................

to solve Predict function:

write gPrime ---> a
update ekfCov ---> b

From Estimation for Quadrotors PDF section 7.2 Transition equation (51) https://www.overleaf.com/read/vymfngphcccj :
    ........(ones).........
 	gPrime(0, 0) = 1; ---> a
	gPrime(1, 1) = 1; ---> a
	gPrime(2, 2) = 1; ---> a
	gPrime(3, 3) = 1; ---> a
	gPrime(4, 4) = 1; ---> a
	gPrime(5, 5) = 1; ---> a
    gPrime(6, 6) = 1; ---> a

  ...........(dt)..........
  gPrime(0, 3) = dt; ---> a
  gPrime(1, 4) = dt; ---> a
  gPrime(2, 5) = dt; ---> a

  ...............(everything else)...............
  gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt; ---> a
  gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt; ---> a
  gPrime(5, 6) = 0; ---> a // zero because RbgPrime(2) is zero,zero,zero

  ...................................(ekfCov)........................................
  ekfCov = (old ekfCove)*(transpose gPrime)+  transition model covariance(Q);
  ekfCov = ekfCov * gPrime.transpose() + Q; ---> b

  <----- *(Fourth correction)* ----->
  (((((((((((((((((((
  From udacity reviw (report):
  	gPrime(0, 3) = gPrime(1, 4) = gPrime(2, 5) = dt;

	VectorXf accelV(3);
	accelV << accel[0], accel[1], accel[2];
	VectorXf term = RbgPrime * accelV;
	term *= dt;

	gPrime(3, 6) = term[0];
	gPrime(4, 6) = term[1];
	gPrime(5, 6) = term[2];

	ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

  )))))))))))))))))))))

  <----- *(Fifth correction)* ----->
  Change from QuadControlParams
  # Angle rate gains
kpPQR = 29(was 69), 29(was 69), 15

  Change from QuadEstimatorEKE.txt
  QYawStd = 0.2(was 0.005)