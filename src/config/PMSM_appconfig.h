/**********************************************************************/
// File Name: {FM_project_loc}/PMSM_appconfig.h 
//
// Date:  21. October, 2024
//
// Automatically generated file by MCAT
// - static configuration of the PMSM FOC application
/**********************************************************************/

#ifndef __PMSM_APPCONFIG_H
#define __PMSM_APPCONFIG_H


//Motor Parameters                      
//----------------------------------------------------------------------
//Pole-pair number                      = 2 [-]
//Stator resistance                     = 0.192 [Ohms]
//Direct axis inductance                = 0.000096 [H]
//Quadrature axis inductance            = 0.000107 [H]
//Back-EMF constant                     = 0.00587 [V.sec/rad]
//Drive inertia                         = 0.12e-4 [kg.m2]
//Nominal current                       = 6 [A]
//Nominal speed                         = 5100 [rpm]

#define MOTOR_PP                        (2.0F)
#define MOTOR_PP_GAIN                   FRAC32(0.5)
#define MOTOR_PP_SHIFT                  (2)

//Application scales                    
//----------------------------------------------------------------------
#define I_MAX                           (15.625F)
#define U_DCB_MAX                       (22.1F)
#define N_MAX                           (5200.0F)
#define WEL_MAX                         (1089.09F)
#define E_MAX                           (6.765F)

//Application Fault Triggers            
//----------------------------------------------------------------------
#define U_DCB_TRIP                      (17.0F)
#define U_DCB_UNDER                     (8.0F)
#define U_DCB_OVER                      (18.0F)
#define I_PH_OVER                       (7.0F)
#define TEMP_OVER                       (110.0F)

//DC Bus voltage IIR1 filter constants  
//----------------------------------------------------------------------
//Cut-off frequency                     = 100 [Hz]
//Sample time                           = 0.0001 [sec]
//----------------------------------------------------------------------
#define UDCB_IIR_B0                     (0.030459027951421222F)
#define UDCB_IIR_B1                     (0.030459027951421222F)
#define UDCB_IIR_A1                     (-0.9390819440971575F)
//Mechanical Alignment                  
#define ALIGN_VOLTAGE                   (0.5F)
#define ALIGN_DURATION                  (10000)

//Current Loop Control                  
//----------------------------------------------------------------------
//Loop Bandwidth                        = 280 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Current Controller Output Limit       
#define CLOOP_LIMIT                     (0.9F)
//D-axis Controller - Recurrent type    
#define D_CC1SC                         (0.16064056022684633F)
#define D_CC2SC                         (-0.13092752400110277F)
//Q-axis Controller - Recurrent type    
#define Q_CC1SC                         (0.20104729108617247F)
#define Q_CC2SC                         (-0.16792963612622913F)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Loop Bandwidth                        = 5 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.001 [sec]
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#define SPEED_PI_PROP_GAIN              (0.04281557279168372F)
#define SPEED_PI_INTEG_GAIN             (0.0003362727223539815F)
#define SPEED_LOOP_HIGH_LIMIT           (6.0F)
#define SPEED_LOOP_LOW_LIMIT            (-6.0F)

#define SPEED_RAMP_UP                   (0.62832F)
#define SPEED_RAMP_DOWN                 (0.62832F)

#define SPEED_LOOP_CNTR                 (10)

#define POSPE_SPEED_FILTER_MA_LAMBDA    (0.8F)

//Position & Speed Sensors Module       
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//Encoder Loop Bandwidth                = 100 [Hz]
//Encoder Loop Attenuation              = 1 [-]
#define POSPE_ENC_TO_CC1                (1276.376270238096F)
#define POSPE_ENC_TO_CC2                (-1236.8978526337387F)
#define POSPE_ENC_TO_INTEG_GAIN         (0.00005F)
#define ENC_PULSES                      (1024)

//Sensorless DQ BEMF Observer and Tracking Observer
//----------------------------------------------------------------------
//Loop Bandwidth                        = 280 [Hz]
//Loop Attenuation                      = 1 [-]
//Loop sample time                      = 0.0001 [sec]
//----------------------------------------------------------------------
//DQ Bemf - plant coefficients          
#define I_Gain                          (0.8181818181818181F)
#define U_Gain                          (0.4734848484848485F)
#define E_Gain                          (0.4734848484848485F)
#define WI_Gain                         (0.00005066287878787879F)

//DQ Bemf - PI controller parameters    
#define BEMF_DQ_CC1_GAIN                (0.16064056022684633F)
#define BEMF_DQ_CC2_GAIN                (-0.13092752400110277F)

//Tracking Observer - PI controller parameters
#define TO_CC1SC                        (252.1169806392706F)
#define TO_CC2SC                        (-250.5378439350963F)
//Tracking Observer - Integrator        
#define TO_THETA_GAIN                   (0.00005F)

//Observer speed output filter          

//Open loop start-up                    
#define OL_START_RAMP_INC               (0.20944F)
#define OL_START_I                      (3.0F)
#define MERG_SPEED_1_TRH                (300.0F)
#define MERG_SPEED_2_TRH                (600.0F)

//Control Structure Module - Scalar Control
//----------------------------------------------------------------------
#define SCALAR_VHZ_FACTOR_GAIN          (0.017675560738676377F)
#define SCALAR_INTEG_GAIN               FRAC32(0.017333333333)
#define SCALAR_INTEG_SHIFT              (0)
#define SCALAR_RAMP_UP                  (0.06283F)
#define SCALAR_RAMP_DOWN                (0.06283F)

//FreeMASTER Scale Variables            
//----------------------------------------------------------------------
//Note: Scaled at input = 1000          
//----------------------------------------------------------------------
#define FM_SPEED_RAD_MEC_SCALE          (1000)
#define FM_SPEED_RPM_EL_SCALE           (9549)
#define FM_SPEED_RPM_MEC_SCALE          (4775)
#define FM_POSITION_DEG_SCALE           (57296)

#endif

//End of generated file                 
/**********************************************************************/
