#ifndef GPIOSET_H
#define GPIOSET_H

#ifdef __cplusplus
extern "C" {
#endif


void initGpio(int n);
void setGpioDirection(int n,char *direction);
int getGpioValue(int n);
unsigned int getGpioValueMutiple(int n,int num);
int gpio_level_set(const int pin,unsigned int flag);

#ifdef __cplusplus
}
#endif

#endif // GPIOSET_H
