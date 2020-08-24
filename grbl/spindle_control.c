/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
#endif

// ESC must be inited and warm up after it's power on
// after connection , ESC must run at a lower speed (warm up) for a little time first, 
// before it can do high PWM high RPM output
// this is a safety feature for many ESC, to prevent it from connected and immediatley run like crazy
void initEsc(void)
{
#ifdef DebugESC
    //report_data_value_float(PSTR("ESC_PWM_CYCLE_TIME_US"), ESC_PWM_CYCLE_TIME_US);
    //report_data_value_uint8(PSTR("init ESC"), ESC_SPINDLE_PWM_MIN);
#endif
    spindle_set_speed(ESC_SPINDLE_PWM_MIN); // using min pwm signal to arm the esc
    
    SPINDLE_TCCRA_REGISTER |= (1 << SPINDLE_COMB_BIT); // Ensure PWM output is enabled.

    // must give some time  to ESC to establish connection with arduino
    delay_ms(800);
}


// min pwm on time :1000us = 1 ms   max pwm on time: 2000us = 2 ms
uint8_t convertNormalPwmToEscPwm(uint8_t pwm)
{
#ifdef DebugESC    
    // report_data_value_uint8(PSTR("input pwm to convert"), pwm);
#endif
    // this is basically 0 output, to simplify things, just output the Min ESC PWM value instead of calculating to the end.
    const uint8_t MinRegPWM = 1;
    if (pwm <= MinRegPWM)
    {
        return ESC_SPINDLE_PWM_MIN;
    }
    else if (pwm >= (ARDUINO_PWM_MAX-1))
    {
        // for max regular PWM like 254 or 255, simply return the max esc pwm
        return ESC_SPINDLE_PWM_MAX;
    }

    // cache last value use it next time.
    static uint8_t prev_pwm = 0;
    static uint8_t prev_esc_pwm = ESC_SPINDLE_PWM_MIN;
    if (prev_pwm == pwm)
    {
        // this is the same input as last time, 
        // so we just return the same result of last time.
        #ifdef DebugESC    
        report_data_value_uint8(PSTR("skip convert, escpwm:"), prev_esc_pwm);
        #endif
        return prev_esc_pwm;
    }
    // make it float so later we won't cause int calculation problem.
    const float OneSec1000Ms = 1000.0f; //1sec =1000ms
    float powerPercent = (pwm * 1.0f / ARDUINO_PWM_MAX);
    //Serial.println("inputPwmPercent " + String(powerPercent));
    // must use US not to loss data
    int onTimeUs = round((ESC_SPINDLE_PWM_ONTIMEUS_MIN_US + ((ESC_SPINDLE_PWM_ONTIMEUS_MAX_US - ESC_SPINDLE_PWM_ONTIMEUS_MIN_US) * powerPercent)));
    //Serial.println("PWM: " + String(pwm) + " to: " + String(onTimeUs) + " On time US");
    const unsigned int PwmCycleTimeUs = round(OneSec1000Ms * 1000.0 / ESC_PWM_FREQ); //one pwm cycle for 60hz is 1sec/60hz =16.6ms=16666us 
    //Serial.println("PwmCycleTimeUs " + String(PwmCycleTimeUs));

    int escPWM = round((onTimeUs * 1.0f) / PwmCycleTimeUs * ARDUINO_PWM_MAX);
    // cache the input and output value for next time for a super fast calculation.
    prev_pwm = pwm;
    prev_esc_pwm = escPWM;
    return escPWM;
}


void spindle_init()
{
  #ifdef VARIABLE_SPINDLE
    // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
    // combined unless configured otherwise.
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
    if (SpindleUsingESC)
    {
       // To support Brushed or Brushless Motor work with an external Brushed or Brushless ESC.
       // they use 50Hz PWM signal to control speed or angle. on period 1ms = min, 2ms=Max
       // this is the same for servo angle control.
       // using ESC to control motor       
       //SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK_60HZ;
       //SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK_244HZ;  // 244 is the best option
       //SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK_488HZ;
       SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK_244HZ;
       initEsc();
    }
    else
    {
       // default PWM 0% - 100% controlled motor.
       SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
    }
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #else
      #ifndef ENABLE_DUAL_AXIS
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
      #endif
    #endif
    pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #ifndef ENABLE_DUAL_AXIS
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    #endif
  #endif

  spindle_stop();
}


uint8_t spindle_get_state()
{
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      // No spindle direction output pin. 
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #else
        if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #endif
    #else
      if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
        #ifdef ENABLE_DUAL_AXIS
          return(SPINDLE_STATE_CW);
        #else
          if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        #endif
      }
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { 
    #else
      if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
    #endif
      #ifdef ENABLE_DUAL_AXIS    
        return(SPINDLE_STATE_CW);
      #else
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      #endif
    }
  #endif
  return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
  #ifdef VARIABLE_SPINDLE
   if (SpindleUsingESC)
   {
      // using ESC to control motor
      // Freq is fixed 50HZ
      // Min PWM 1MS (Stop), Max PWM 2MS
      // using the min pwm for esc to stop ( 0 pwm just cause ESC problem )
        #ifdef DebugESC
         printPgmString(PSTR("stop\r\n"));
        #endif
      // this does a sudden brake when using ESC, need to avoid
      // spindle_set_speed(ESC_SPINDLE_PWM_MIN);
      // this does a slowly slow down.
      esc_pwm_change(ESC_SPINDLE_PWM_MIN);
      SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT); // Set pin to low
      return;
   }
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif
}


#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(uint8_t pwm_value)
  {
    SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
    // do not disable PWM output when using ESC, it would be disconnected.
    if(SpindleUsingESC)
    {
       // pwm value has been set, return now
       return;
    }
    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        spindle_stop();
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      }
    #else
      if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
      } else {
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      }
    #endif
  }


  #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
        rpm = RPM_MAX;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= RPM_MIN) {
        if (rpm == 0.0) { // S0 disables spindle
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else {
          rpm = RPM_MIN;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else {
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        #if (N_PIECES > 3)
          if (rpm > RPM_POINT34) {
            pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
          } else 
        #endif
        #if (N_PIECES > 2)
          if (rpm > RPM_POINT23) {
            pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
          } else 
        #endif
        #if (N_PIECES > 1)
          if (rpm > RPM_POINT12) {
            pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
          } else 
        #endif
        {
          pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
        }
      }
      sys.spindle_speed = rpm;
      return(pwm_value);
    }
    
  #else 
  
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0) { // S0 disables spindle
          sys.spindle_speed = 0.0;
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else { // Set minimum PWM output
          sys.spindle_speed = settings.rpm_min;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else { 
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
    }

    if (SpindleUsingESC)
    {
        #ifdef DebugESC
        //report_data_value_uint8(PSTR("converting pwm"), pwm_value);
        #endif
        pwm_value = convertNormalPwmToEscPwm(pwm_value);
        #ifdef DebugESC
        //report_data_value_uint8(PSTR("converted esc pwm"), pwm_value);
        #endif
    }
    return(pwm_value);
}
 
  #endif
#endif

#ifdef SpindleUsingESC
// use it to "slowly" change the speed, could be speeding up or slow down
// it's necessary to "slowly" to speed up because most ESC would reject a sudden jump from 0 to 255 pwm signal.
// for slowing down, if we suddenly send a stop signal (1ms) when it's in high pwm output, 
// the ESC does a very hard brake and stop on the motor, which is not necessary and could potentially cause problems.
// so we need to "slowly" slow down as well, maybe not so slow comparing to speeding up.
void esc_pwm_change(uint8_t pwm)
{
    // if it's speeding up
    if (pwm == SPINDLE_CURRENT_ESC_PWM)
    {
      // SPINDLE_CURRENT_ESC_PWM is register, no need setting again.
      // same pwm, do nothing.
      return;
    }
    // validate pwm value first
    if (pwm < ESC_SPINDLE_PWM_MIN)
    {
       pwm = ESC_SPINDLE_PWM_MIN;
    }
    else if (pwm > ESC_SPINDLE_PWM_MAX)
    {
       pwm = ESC_SPINDLE_PWM_MAX;
    }
    #ifdef DebugESC
      //report_data_value_uint8(PSTR("cur pwm"), SPINDLE_CURRENT_ESC_PWM);
    #endif
    // the final pwm is too large and it will cause the ESC disconnected,
    const uint8_t MaxSteps = 10; // at most we use 6 steps to speed up the spindle from 0 to max pwm.
    int16_t PwmStep = (ESC_SPINDLE_PWM_MAX - ESC_SPINDLE_PWM_MIN) / MaxSteps; //each step we increase / decrease pwm
    if (PwmStep <= 0)
    {
       PwmStep = 1; //mininum step increase should at least be 1, must not be 0.
    }
    const uint16_t StepDelayMs = 200;
    int16_t temp_pwm = 0; // use int16 so we don't overflow and it could be negative
    uint8_t setPwm = 0;
    if (pwm > SPINDLE_CURRENT_ESC_PWM)
    {   
        // this is for speeding up
        do
        {
           temp_pwm = PwmStep + SPINDLE_CURRENT_ESC_PWM;
           if (temp_pwm > pwm)
           {
              temp_pwm = pwm;
           }            
           //printPgmString(PSTR("temp_pwm"));
           //print_uint32_base10(temp_pwm);
           setPwm = (uint8_t)(temp_pwm % 0x100);
           //printPgmString(PSTR("setPwm"));
           //print_uint32_base10(setPwm);
           #ifdef DebugESC
           report_data_value_uint8(PSTR("SetEsPm:"), setPwm);
           #endif
           spindle_set_speed(setPwm);
           #ifdef DebugESC
           printPgmString(PSTR("delay1\n"));
           #endif
           delay_ms(StepDelayMs);
        } while (setPwm < pwm);
    }
    else // speed down
    {    
       do
       {
          temp_pwm = SPINDLE_CURRENT_ESC_PWM - PwmStep;
          if (temp_pwm < pwm)
          {
             temp_pwm = pwm;
          }
          setPwm = (uint8_t)(temp_pwm % 0x100);
          #ifdef DebugESC
          report_data_value_uint8(PSTR("Setting down Esc Pwm:"), setPwm);
          #endif
            spindle_set_speed(setPwm);
          #ifdef DebugESC
            printPgmString(PSTR("delay1\n"));
          #endif
            delay_ms(StepDelayMs);
        } while (setPwm > pwm);
    } // end of if speed up else speed down
} // end of void esc_pwm_change(uint8_t pwm)


#endif

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } // Block during abort.

  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {
    
    #if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(ENABLE_DUAL_AXIS)
      if (state == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif
  
    #ifdef VARIABLE_SPINDLE
      // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
      if (settings.flags & BITFLAG_LASER_MODE) { 
        if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
      }

        // For ESC, must slowly increase the RPM for the first time after power up, 
        // must not do it from 0 to max just after power up
        if (SpindleUsingESC)
        {             
           #ifdef DebugESC
             report_data_value_float(PSTR("spindle_set_state"), rpm);
             report_data_value_float(PSTR("current running rpm"), sys.spindle_speed);
           #endif
            
           uint8_t esc_pwm = spindle_compute_pwm_value(rpm);
           #ifdef DebugESC
              report_data_value_uint8(PSTR("setting esc pwm"), esc_pwm);
           #endif
           esc_pwm_change(esc_pwm);
            
        } // if (SpindleUsingESC)
        else // the regular PWM motor path, not using ESC.
        {
            spindle_set_speed(spindle_compute_pwm_value(rpm));
        } // if (SpindleUsingESC)
        
   #endif
    #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
        !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif    
    #endif
  
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif
