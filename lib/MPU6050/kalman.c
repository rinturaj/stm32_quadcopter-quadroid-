#include "kalman.h"

 void    the_Init (on_kalman *klm) {
      /* We at The Will SET the Variables like SO, THESE CAN BE Also at The Tuned by the User */ 
     klm -> Q_angle = 0.001 ;
     klm -> Q_bias = 0.003 ;
     klm-> R_measure = 0.03 ;
 
     klm-> Angle = 0 ; // the Reset at The Angle 
     klm-> BIAS = 0 ; // the Reset BIAS 
 
     klm-> P [ 0 ] [ 0 ] = 0 ; // Since the ASSUME that We at The BIAS IS 0 and We know at The Starting Angle (use setAngle), at The error covariance Matrix like SO IS SET - See: HTTP: // en- > wikipedia-> ORG / Wiki / Kalman_filter # Example_application-> 2C_technical 
     klm-> P [ 0 ] [ 1 ] = 0 ;
     klm-> P [ 1 ] [ 0 ] = 0 ;
     klm-> P [ 1 ] [ 1 ] = 0 ;
 }
 
 // at The Angle SHOULD BE in Degrees and Degrees at The Rate in per SHOULD BE SECOND and at The Delta Time in seconds The 
 double getAngle ( on_kalman *klm, double newAngle, double newRate, double dt) {
     // KasBot V2 - the filter on Kalman Module1 - HTTP: // www- ?> X-firm-> COM / 145 page_id =
     // Modified by Kristian Lauszus
     // See My Blog POST for More Information: HTTP: // blog- > tkjelectronics -> DK / 2012/09 / A-& PHARMACY-approach-to-Kalman-the filter-and-How-to-Implement-IT
     
     // Discrete on Kalman the filter Update the Equations Time - Time the Update ( "Predict")
     // xhat the Update - the Project at The State Ahead 
     // * the Step 1 * / 
     klm-> Rate = newRate - klm-> BIAS;
     klm-> Angle +=  dt * klm-> Rate;
     
     // the Update Estimation error covariance - Ahead at The error covariance the Project 
//64-      // * the Step 2 * / 
     klm-> P [ 0 ] [ 0 ] +=  (dt * (klm-> P [ 1 ] [ 1 ] - klm-> P [ 0 ] [ 1 ] - klm-> P [ 1 ] [ 0 ] + klm-> Q_angle));
     klm-> P [ 0 ] [ 1 ] -= dt * klm-> P [ 1 ] [ 1 ];
     klm-> P [ 1 ] [ 0 ] -= dt * klm-> P [ 1 ] [ 1 ];
     klm-> P [ 1 ] [ 1 ] += klm-> Q_bias * dt;
     
     // Discrete Measurement Update the filter on Kalman the Equations - the Measurement the Update ( "Correct")
     // the Calculate on Kalman GAIN - the Compute on Kalman at The GAIN 
     // * the Step 4 * / 
     klm-> S = klm-> P [ 0 ] [ 0 ] + klm-> R_measure;
     // * the Step 5 * / 
     klm-> K [ 0 ] = klm-> P [ 0 ] [ 0 ] / klm-> S;
     klm-> K [ 1 ] = klm-> P [ 1 ] [ 0 ] / klm-> S;
     
     // the Calculate Angle and BIAS - Estimate the with the Update Measurement ZK (newAngle) 
     // * the Step 3 * / 
     klm-> Y = newAngle - klm-> Angle;
     //* the Step. 6 * / 
     klm-> Angle += klm-> K [ 0 ] * klm-> Y;
     klm-> BIAS += klm-> K [ 1 ] * klm-> Y;
     
     // the Calculate Estimation error covariance - the Update error covariance at The 
     // * the Step. 7 * / 
     klm-> P [ 0 ] [ 0 ] -= klm-> K [ 0 ] * klm-> P [ 0 ] [ 0 ];
     klm-> P [ 0 ] [ 1 ] -= klm-> K [ 0 ] * klm-> P [ 0 ] [ 1 ];
     klm-> P [ 1 ] [ 0 ] -= klm-> K [ 1 ] * klm-> P [ 0 ] [ 0 ];
     klm-> P [ 1 ] [ 1 ] -= klm-> K [ 1 ] * klm-> P [ 0 ] [ 1 ];
    
     return klm-> Angle;
 }
 
 
 
