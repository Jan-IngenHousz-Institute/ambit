#ifndef __u_MLX_H_
#define __u_MLX_H_


#ifdef __cplusplus
extern "C" {
#endif
#include "mlx90632.h"

#ifdef __cplusplus
}
#endif /* End of CPP guard */

//#include "mlx90632.c"
//#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include "../pin_config.h"


bool mlx_init(void);
double mlx_measure(double* object, double* ambient);
double mlx_measure();



#endif