/*********************
 *      INCLUDES
 *********************/

#include "lvgl_port.h"
#include "lvgl/lvgl.h"
#include "stm32u5xx_hal.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

void touch_read(lv_indev_t * indev, lv_indev_data_t * data);

/**********************
 *  STATIC VARIABLES
 **********************/

static volatile bool do_sample_touch = false;
static lv_indev_state_t last_state = LV_INDEV_STATE_RELEASED;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lvgl_port_init(void)
{
    lv_init();

    lv_tick_set_cb(HAL_GetTick);

    static __attribute__((aligned(32))) uint8_t buf_2[800 * 480 * 2];
    // static __attribute__((aligned(32))) uint8_t buf_3[800 * 480];
    lv_st_ltdc_create_direct((void *)0x20000000, buf_2, 0);
    // lv_st_ltdc_create_partial(buf_2, buf_3, 800 * 480, 0);

    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TP_IRQ_Pin)
    {
        /* Communication with TS is done via I2C.
        Often the sw requires ISRs (interrupt service routines) to be quick while communication
        with I2C can be considered relatively long (depending on SW requirements).
        Considering that the TS feature don't need immediate reaction,
        it is suggested to use polling mode instead of EXTI mode,
        in order to avoid blocking I2C communication on interrupt service routines */

        /* Here an example of implementation is proposed which is a mix between pooling and exit mode:
        On ISR a flag is set (exti5_received), the main loop polls on the flag rather then polling the TS;
        Mcu communicates with TS only when the flag has been set by ISR. This is just an example:
        the users should choose they strategy depending on their application needs.*/

        do_sample_touch = true;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

void touch_read(lv_indev_t * indev, lv_indev_data_t * data)
{
    NVIC_DisableIRQ(EXTI5_IRQn);
    if (do_sample_touch)
    {
        uint8_t touches = 0;
        uint8_t buf[6];
        const uint16_t STATUS_REG = 0x814E;
        const uint16_t TOUCH_POS_REG = 0x8150;
        uint8_t ZERO = 0;

        HAL_I2C_Mem_Read(&hi2c2, 0xBA, STATUS_REG, 2, buf, 1, HAL_MAX_DELAY);
        touches = (0x0F & buf[0]);

        HAL_I2C_Mem_Write(&hi2c2, 0xBA, STATUS_REG, 2, &ZERO, 1, HAL_MAX_DELAY);

        do_sample_touch = false;

        if (touches > 0)
        {
            last_state = LV_INDEV_STATE_PRESSED;

            HAL_I2C_Mem_Read(&hi2c2, 0xBA, TOUCH_POS_REG, 2, buf, 4, HAL_MAX_DELAY);
            data->point.x = buf[0] + (buf[1] << 8);
            data->point.y = buf[2] + (buf[3] << 8);
        }
        else {
            last_state = LV_INDEV_STATE_RELEASED;
        }
    }
    NVIC_EnableIRQ(EXTI5_IRQn);

    data->state = last_state;
}
