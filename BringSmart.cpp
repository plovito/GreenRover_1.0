#include "BringSmart.h"

void BringSmartMotor::init(int _FE, int _SE, int _FM, int _SM) {

  Timer = millis();

  FE = _FE;
  SE = _SE;
  FM = _FM;
  SM = _SM;

  F = 0;
  S = 0;

  HardPositionPrev = 0;
  HardPosition = 0;
  HardVelocity = 0;

  pinMode(FE, INPUT_PULLUP);
  pinMode(SE, INPUT_PULLUP);
  pinMode(FM, OUTPUT);
  pinMode(SM, OUTPUT);

  drv_pwm_set_freq(FM, PWM_Freq);
  drv_pwm_setup(FM);
  drv_pwm_set_freq(SM, PWM_Freq);
  drv_pwm_setup(SM);

  
//
//    pTIM = g_Pin2PortMapArray[FM].TIMx;
//    tim_ch = g_Pin2PortMapArray[FM].timerChannel;
//    if (tim_ch == TIM_CHANNEL_1) {
//      pOC = &hOC1;
//    }
//    if (tim_ch == TIM_CHANNEL_2) {
//      pOC = &hOC2;
//    }
//    if (tim_ch == TIM_CHANNEL_3) {
//      pOC = &hOC3;
//    }
//
//    GPIO_InitTypeDef GPIO_InitStruct;
//
//    __HAL_RCC_TIM1_CLK_ENABLE();
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//
//    GPIO_InitStruct.Pin = GPIO_PIN_9;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//    memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));
//
//    pOC->OCMode = TIM_OCMODE_PWM1;
//    pOC->OCPolarity = TIM_OCPOLARITY_HIGH;
//    pOC->OCFastMode = TIM_OCFAST_DISABLE;
//    pOC->OCNPolarity = TIM_OCNPOLARITY_HIGH;
//    pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
//    pOC->OCIdleState = TIM_OCIDLESTATE_RESET;
//
//    pOC->Pulse = 0;
//    HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
//    HAL_TIM_PWM_Start(pTIM, tim_ch);
}

void BringSmartMotor::jointMode() {
  FlagWheelMode = false;
  FlagJointMode = true;
}

void BringSmartMotor::wheelMode() {
  FlagJointMode = false;
  FlagWheelMode = true;
}

void BringSmartMotor::setGoalVelocity(double _GoalRadianVelocity) {
  wheelMode();
  GoalRadianVelocity = _GoalRadianVelocity;
}

void BringSmartMotor::calcRealVelocity() {
  if (millis() - Timer > ENCODER_POLLING_RATE) {
    Timer = millis();
    HardVelocity = HardPosition - HardPositionPrev;
    HardPositionPrev = HardPosition;
    RealRadianVelocity = HardVelocity * SPEED_RATIO * (1000 / ENCODER_POLLING_RATE);

    //Serial.print("RealRadianVelocity: ");
    //Serial.println(HardPosition);
    //Serial.println( HardPositionPrev);

    if (GoalRadianVelocity != 0) {
      velocityPID();
      controlDriver();
      FlagSavePosition = true;
    } else {
      if (FlagSavePosition) {
        GoalPosition = HardPosition;
        FlagSavePosition = false;
      }

      integralPosPrevErr = 0;
      integralVelPrevErr = 0;
      integralVel = 0;
      integralPos = 0;
      PWM = 0;

      jointMode();
      positionPID();
      controlDriver();
    }
  }
}

double BringSmartMotor::getRealRadianVelocity() {
  return RealRadianVelocity;
}

double BringSmartMotor::getGoalRadianVelocity() {
  return GoalRadianVelocity;
}

void BringSmartMotor::tick() {
  calcRealVelocity();
}

void BringSmartMotor::interruptListener() {

  F = digitalRead(FE);
  S = digitalRead(SE);

  if (F == S)
    HardPosition++;
  else
    HardPosition--;

   // Serial.print("HardPosition: ");
    //Serial.println(HardPosition);
  //  F = (GPIOB->IDR & GPIO_PIN_9);
  //  S = (GPIOG->IDR & GPIO_PIN_6);
  //  if (F != 0 && S != 0) HardPosition++;
  //  if (F == 0 && S == 0) HardPosition++;
  //  if (F != 0 && S == 0) HardPosition--;
  //  if (F == 0 && S != 0) HardPosition--;
}

void BringSmartMotor::velocityPID() {

  double err = GoalRadianVelocity - RealRadianVelocity;
  integralVel = constrainPWM(integralVel + (double)err * V_KI * ((double)ENCODER_POLLING_RATE / 1000));
  double D = (err - integralVelPrevErr) / ((double)ENCODER_POLLING_RATE / 1000);
  integralVelPrevErr = err;
  PWM = constrainPWM(err * V_KP + integralVel + D * V_KD);
}

void BringSmartMotor::positionPID() {
  double err = GoalPosition - HardPosition;
  integralPos = constrainPWM(integralPos + (double)err * P_KI * ((double)ENCODER_POLLING_RATE / 1000));
  double D = (err - integralPosPrevErr) / ((double)ENCODER_POLLING_RATE / 1000);
  integralPosPrevErr = err;
  PWM = constrainPWM(err * P_KP + integralPos + D * P_KD);
}

void BringSmartMotor::controlDriver() {
  if (PWM == 0) {
    drv_pwm_set_duty(FM, 8, 0);
    drv_pwm_set_duty(SM, 8, 0);
  }
  if (PWM < 0) {
    drv_pwm_set_duty(FM, 8, 0);
    drv_pwm_set_duty(SM, 8, abs(PWM));
  }
  if (PWM > 0) {
    drv_pwm_set_duty(SM, 8, 0);
    drv_pwm_set_duty(FM, 8, abs(PWM));
  }
  //Serial.print("PWM: ");
  //Serial.println(PWM);
}

double BringSmartMotor::constrainPWM(double value) {
  if (value < MIN_PWM)
    return MIN_PWM;
  else if (value > MAX_PWM)
    return MAX_PWM;
  else if (value > 0 and value < UPPER_BAR_PWN)
    return UPPER_BAR_PWN;
  else if (value < 0 and value > -BOTTOM_BAR_PWM)
    return -BOTTOM_BAR_PWM;
  else return value;
}
