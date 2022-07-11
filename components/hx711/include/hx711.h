#ifndef _HX711_H_
#define _HX711_H_

#include <driver/gpio.h>

typedef enum {
    HX711_GANANCIA_A_128 = 128,
    HX711_GANANCIA_B_32,
    HX711_GANANCIA_A_64,
} ganancia_hx711_t;

typedef struct {
    gpio_num_t pin_dout;
    gpio_num_t pd_sck;
    ganancia_hx711_t ganancia;
} hx711_t;

esp_err_t hx711_init(hx711_t* sensor);

esp_err_t hx711_desactivar(hx711_t* sensor);

esp_err_t hx711_set_ganancia(hx711_t* sensor, ganancia_hx711_t ganancia);

esp_err_t hx711_revisar_si_listo(hx711_t* sensor, bool* estaListo);

esp_err_t hx711_leer_datos(hx711_t* sensor, int32_t* datos);

#endif 