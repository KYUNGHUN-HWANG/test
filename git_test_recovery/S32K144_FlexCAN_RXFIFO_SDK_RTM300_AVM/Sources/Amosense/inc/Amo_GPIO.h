#ifndef Amo_GPIO_H_
#define Amo_GPIO_H_

#define BUTTON_50MS		1
#define BUTTON_100MS	2
#define BUTTON_500MS	10
#define BUTTON_600MS	12
#define BUTTON_700MS	14
#define BUTTON_800MS	16
#define BUTTON_900MS	18
#define BUTTON_1000MS	20
#define BUTTON_1200MS	24
#define BUTTON_1500MS	30
#define BUTTON_1800MS	36
#define BUTTON_2000MS	40
#define BUTTON_3000MS	60
#define BUTTON_4000MS	80
#define BUTTON_5000MS	100
#define BUTTON_6000MS	120
#define BUTTON_7000MS	140
#define BUTTON_8000MS	160

#ifdef	Amo_GPIO_C_

void Amo_Gpio_Init(void);

#ifdef KEY_ENABLE
void Button_Func(void);
void Button_Long_Func(void);
extern void Button_Func(void);
extern void Button_Long_Func(void);
#endif

#else

extern void Amo_Gpio_Init(void);


#endif /* Amo_GPIO_C_ */

extern uint32_t buttonsPressed;
#endif /* Amo_GPIO_H_ */
