
#ifndef ADC_H_
#define ADC_H_

#define CURRENT     BIT0

void ADC_init (void);
void ADC_convert (void);
void HR_Current_Averaging (void);
void HR_Fetch_Current_Values (void);
void BR_Current_Averaging (void);
void BR_Fetch_Current_Values (void);

void Current_Avg(void);

#endif /* ADC_H_ */
