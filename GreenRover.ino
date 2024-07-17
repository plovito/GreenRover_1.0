#include "BringSmart.h"
#include "drv_pwm.h"

BringSmartMotor motor1;
BringSmartMotor motor2;
BringSmartMotor motor3;
BringSmartMotor motor4;
void interruptListener1() {
  motor1.interruptListener();
}
void interruptListener2() {
  motor2.interruptListener();
}
void interruptListener3() {
  motor3.interruptListener();
}
void interruptListener4() {
  motor4.interruptListener();
}

volatile long tim = 0;
volatile long timStop = 0;
int ch1 = 0;
int ch2 = 0;
int ch6 = 0;
int ch1Prev = 0;
int ch2Prev = 0;
int speed = 0;
unsigned long long timer = 0;
bool flagStop = false;

int upBar = 2050;
int downBar = 950;
int time1;

TIM_HandleTypeDef *pTIM;
TIM_OC_InitTypeDef *pOC;
extern TIM_OC_InitTypeDef hOC1;
extern TIM_OC_InitTypeDef hOC2;
extern TIM_OC_InitTypeDef hOC3;
extern TIM_OC_InitTypeDef hOC4;
uint32_t tim_ch = TIM_CHANNEL_2;
uint32_t uwPeriodValue;
uint32_t uwPrescalerValue = 1;
uint32_t tim_clk;
bool is_timer_16bit = true;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {

  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A2, INPUT);

//  digitalRead();
//  digitalWrite();
  analogWrite(6, 0);
  analogWrite(9, 0);


  motor1.init(2, 0, 3, 5);
  motor2.init(4, 1, 6, 9);
  motor3.init(7, 14, 10, 11);
  motor4.init(8, 15, 12, A0);



  attachInterrupt(digitalPinToInterrupt(2), interruptListener1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), interruptListener2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), interruptListener3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), interruptListener4, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(3);
  tim = millis();

  pinMode(5, OUTPUT);
  analogWrite(5, 0);
  pinMode(11, OUTPUT);
  analogWrite(11, 0);


  analogWrite(12, 0);

  drv_pwm_set_freq(A0, PWM_Freq);
  drv_pwm_setup(A0);


  {
    pTIM = g_Pin2PortMapArray[A0].TIMx;
    tim_ch = g_Pin2PortMapArray[A0].timerChannel;
    if (tim_ch == TIM_CHANNEL_1) {
      pOC = &hOC1;
    }
    if (tim_ch == TIM_CHANNEL_2) {
      pOC = &hOC2;
    }
    if (tim_ch == TIM_CHANNEL_3) {
      pOC = &hOC3;
    }

    GPIO_InitTypeDef GPIO_InitStruct;



    uwPeriodValue = (uint32_t)(SystemCoreClock / 600);

    if (is_timer_16bit == true) {
      uwPrescalerValue = (uwPeriodValue / 0xFFFF) + 1;
      uwPeriodValue /= uwPrescalerValue;
    }
    pTIM->Init.Prescaler = uwPrescalerValue - 1;
    pTIM->Init.Period = uwPeriodValue - 1;
    pTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
    pTIM->Init.RepetitionCounter = 0;

    __HAL_RCC_TIM2_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_TIM_PWM_Init(pTIM);

    memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));

    pOC->OCMode = TIM_OCMODE_PWM1;
    pOC->OCPolarity = TIM_OCPOLARITY_HIGH;
    pOC->OCFastMode = TIM_OCFAST_DISABLE;
    pOC->OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
    pOC->OCIdleState = TIM_OCIDLESTATE_RESET;

    pOC->Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
    HAL_TIM_PWM_Start(pTIM, tim_ch);

    //drv_pwm_set_duty(13, 8, 120);
  }

  drv_pwm_set_freq(12, PWM_Freq);
  drv_pwm_setup(12);

  {
    pTIM = g_Pin2PortMapArray[12].TIMx;
    tim_ch = g_Pin2PortMapArray[12].timerChannel;
    if (tim_ch == TIM_CHANNEL_1) {
      Serial.println("AAAAAA");
      pOC = &hOC1;
    }
    if (tim_ch == TIM_CHANNEL_2) {
      pOC = &hOC2;
    }
    if (tim_ch == TIM_CHANNEL_3) {
      pOC = &hOC3;
    }

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_TIM12_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;


    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uwPeriodValue = (uint32_t)(SystemCoreClock / 600);

    if (is_timer_16bit == true) {
      uwPrescalerValue = (uwPeriodValue / 0xFFFF) + 1;
      uwPeriodValue /= uwPrescalerValue;
    }
    pTIM->Init.Prescaler = uwPrescalerValue - 1;
    pTIM->Init.Period = uwPeriodValue - 1;
    pTIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pTIM->Init.CounterMode = TIM_COUNTERMODE_UP;
    pTIM->Init.RepetitionCounter = 0;


    HAL_TIM_PWM_Init(pTIM);
    memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));


    pOC->OCMode = TIM_OCMODE_PWM1;
    pOC->OCPolarity = TIM_OCPOLARITY_HIGH;
    pOC->OCFastMode = TIM_OCFAST_DISABLE;
    pOC->OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
    pOC->OCIdleState = TIM_OCIDLESTATE_RESET;

    pOC->Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(pTIM, pOC, tim_ch);
    HAL_TIM_PWM_Start(pTIM, tim_ch);



    //drv_pwm_set_duty(12, 8, 120);
  }
  while (ch2 < 950) {
    ch2 = pulseIn(A4, HIGH, 30000);

  }

  tim = millis();
  timStop = millis();
  time1 = millis();

  Serial.print("tempVel, count");
}

float diff1 = 0;
int count1 = 0;
float tempVel1 = 0;
float speed1 = 0;

float diff2 = 0;
int count2 = 0;
float tempVel2 = 0;
float speed2 = 0;


void loop() {
  
ch6 = pulseIn(A2, HIGH, 30000);
if (ch6 > 1800) {
  ch1 = pulseIn(A3, HIGH, 30000);
  ch2 = pulseIn(A4, HIGH, 30000);
  ch2 = 1500 - ch2;
  ch2 += 1500;
    
    
  if (ch2 < 2100 or millis() - timStop < 3000) {
    if (millis() - tim > 50) {
      tim = millis();
      timStop = millis();
      flagStop = false;
 
      if (ch1 < 1510 && ch1 > 1490) ch1 = 1500;
      if (ch2 < 1510 && ch2 > 1490) ch2 = 1500;
      if (ch1 < 300) ch1 = ch1Prev;
      if (ch2 > 2700) ch2 = ch2Prev;
      if (ch2 > 2080) ch2 = 1500;


      if (abs(ch1Prev - ch1) > 5) {
        count1 = 0;
        speed1 = mapfloat(ch1, downBar, upBar, 255, -255);
        //speed1 = 0;
      }


      if (speed1 == 0 ) {
        drv_pwm_set_duty(3, 8, 0);
        drv_pwm_set_duty(5, 8, 0);

        drv_pwm_set_duty(6, 8, 0);
        drv_pwm_set_duty(9, 8, 0);
        ;
      }

      if (speed1 > 0) {
        drv_pwm_set_duty(3, 8, 0);
        drv_pwm_set_duty(5, 8, abs(speed1));

        drv_pwm_set_duty(6, 8, 0);
        drv_pwm_set_duty(9, 8, abs(speed1));
      }

      if (speed1 < 0) {
        drv_pwm_set_duty(5, 8, 0);
        drv_pwm_set_duty(3, 8, abs(speed1));

        drv_pwm_set_duty(9, 8, 0);
        drv_pwm_set_duty(6, 8, abs(speed1));
      }

      if (abs(ch2Prev - ch2) > 5) {
        count2 = 0;
        speed2 = mapfloat(ch2, downBar, upBar, 255, -255);
        //speed2 = 0;

      }


      if (speed2 == 0 ) {
        drv_pwm_set_duty(10, 8, 0);
        drv_pwm_set_duty(11, 8, 0);

        drv_pwm_set_duty(12, 8, 0);
        drv_pwm_set_duty(A0, 8, 0);
        ;
      }

      if (speed2 > 0) {
        drv_pwm_set_duty(10, 8, 0);
        drv_pwm_set_duty(11, 8, abs(speed2));

        drv_pwm_set_duty(12, 8, 0);
        drv_pwm_set_duty(A0, 8, abs(speed2));
      }

      if (speed2 < 0) {
        drv_pwm_set_duty(11, 8, 0);
        drv_pwm_set_duty(10, 8, abs(speed2));

        drv_pwm_set_duty(A0, 8, 0);
        drv_pwm_set_duty(12, 8, abs(speed2));
      }


      ch1Prev = ch1;
      ch2Prev = ch2;






      Serial.print(speed1);
      Serial.print(" , ");
      Serial.println(speed2);

    
    }
   

  } else {
    
    
    //    if (!flagStop) {
    //      flagStop = true;


    Serial.println("STOP");

    drv_pwm_set_duty(10, 8, 0);
    drv_pwm_set_duty(11, 8, 0);

    drv_pwm_set_duty(12, 8, 0);
    drv_pwm_set_duty(A0, 8, 0);

    drv_pwm_set_duty(3, 8, 0);
    drv_pwm_set_duty(5, 8, 0);

    drv_pwm_set_duty(6, 8, 0);
    drv_pwm_set_duty(9, 8, 0);

    //    }
  }
   if ((millis() - time1 > AFK_timer) && speed1 == 0 && speed2 == 0){
      tone(31, 554, 200); 
     time1 = millis(); 
   }
   else if (millis() - time1 > AFK_timer) time1 = millis();
}
else {
  drv_pwm_set_duty(10, 8, 0);
    drv_pwm_set_duty(11, 8, 0);

    drv_pwm_set_duty(12, 8, 0);
    drv_pwm_set_duty(A0, 8, 0);

    drv_pwm_set_duty(3, 8, 0);
    drv_pwm_set_duty(5, 8, 0);

    drv_pwm_set_duty(6, 8, 0);
    drv_pwm_set_duty(9, 8, 0);
   if (millis() - timStop > AFK_timer) { 
      tone(31, 554, 200); 
     timStop = millis();
    } else {
      return;
    }  

   
} 
 
}
