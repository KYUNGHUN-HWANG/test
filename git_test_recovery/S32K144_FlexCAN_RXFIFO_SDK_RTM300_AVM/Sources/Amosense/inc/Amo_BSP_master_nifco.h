#ifndef Amo_BSP_master_nifco_H_
#define Amo_BSP_master_nifco_H_


#ifdef KEY_ENABLE
#define BTN_GPIO        PTC
#define LH_BTN        	0U
#define RH_BTN        	1U
#define BTN_PORT        PORTC
#define BTN_PORT_IRQn           	(PORTC_IRQn)

#define	LH_FACING_INDI_PWM_INDEX	(8UL)
#define	LH_FACING_INDI_PWM_MASK		(0x1u << LH_FACING_INDI_PWM_INDEX)

#define LH_AVOID_INDI_PWM_INDEX		(5UL)
#define LH_AVOID_INDI_PWM_MASK		(0x1u << LH_AVOID_INDI_PWM_INDEX)

#define	RH_FACING_INDI_PWM_INDEX	(4UL)
#define	RH_FACING_INDI_PWM_MASK		(0x1u << RH_FACING_INDI_PWM_INDEX)

#define	RH_AVOID_INDI_PWM_INDEX		(3UL)
#define	RH_AVOID_INDI_PWM_MASK		(0x1u << RH_AVOID_INDI_PWM_INDEX)

#define PORT_BTN0_INDEX         	(0UL)
#define PORT_BTN0_MASK          	(0x1u << PORT_BTN0_INDEX)

#define PORT_BTN1_INDEX         	(1UL)
#define PORT_BTN1_MASK        		(0x1u << PORT_BTN1_INDEX)
#endif

#define GPIO_PORTA             		(PTA)
#define GPIO_PORTB       			(PTB)
#define GPIO_PORTD           		(PTD)
#define GPIO_PORTC       			(PTC)
#define GPIO_PORTE 			    	(PTE)

#define TEST_OUT1_INDEX            (6UL)
#define TEST_OUT1_MASK            ((uint32_t)0x1u << TEST_OUT1_INDEX)

#define TEST_OUT2_INDEX           	(13UL)
#define TEST_OUT2_MASK          	((uint32_t)0x1u << TEST_OUT2_INDEX)

#define	CAN_WAKE_INDEX				(1UL)
#define	CAN_WAKE_MASK				(0x1u << CAN_WAKE_INDEX)

#define CAN_STB_INDEX         		(16UL)
#define CAN_STB_MASK         		((uint32_t)0x1u << CAN_STB_INDEX)

#define	CAN_EN_INDEX				(15UL)
#define	CAN_EN_MASK					((uint32_t)0x1u << CAN_EN_INDEX)

#define LIN1_ENABLE_PIN_INDEX       (7UL)
#define LIN1_ENABLE_MASK      		(0x1u << LIN1_ENABLE_PIN_INDEX)

#define LIN2_ENABLE_PIN_INDEX       (13UL)
#define LIN2_ENABLE_MASK      		((uint32_t)0x1u << LIN2_ENABLE_PIN_INDEX)

//#define LIN3_ENABLE_PIN_INDEX       (2UL)
//#define LIN3_ENABLE_MASK      		(0x1u << LIN3_ENABLE_PIN_INDEX)


#endif /* Amo_BSP_master_nifco_H_ */

