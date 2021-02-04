/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 * @brief TFT Example Application main file.
 *
 * This file contains the source code for a sample application using the
 * GFX library based on the ILI9341 controller.
 *
 */

#include "nrf_gfx.h"
#include "nrf52_dk.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include <math.h>
#include "lvgl/lvgl.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern const nrf_lcd_t nrf_lcd_st7735;
static const nrf_lcd_t * p_lcd = &nrf_lcd_st7735;

// lvgl

lv_obj_t *label;
lv_obj_t *btn1;
lv_obj_t *btn2;

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX / 10];                     /*Declare a buffer for 1/10 screen size*/

lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
lv_indev_drv_t indev_drv;                  /*Descriptor of a input device driver*/

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    p_lcd->lcd_addr_window(area->x1, area->y1, area->x2, area->y2);
    size_t buf_len = (area->y2+1 - area->y1) * (area->x2+1 - area->x1) * 2;
    uint8_t buffer[buf_len];

    int32_t x, y, i = 0;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            buffer[i++] = color_p->full >> 8;
            buffer[i++] = color_p->full;
            color_p++;
        }
    }
    p_lcd->lcd_screen_flush(buffer, buf_len);

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

bool my_touchpad_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    // data->state = touchpad_is_pressed() ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    // if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&data->point.x, &data->point.y);

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

static void event_handler_btn(lv_obj_t * obj, lv_event_t event){
    if(event == LV_EVENT_CLICKED) {
        if (obj == btn1)
        lv_label_set_text(label, "Hello");
        else if (obj == btn2){
          lv_label_set_text(label, "Goodbye");
        }
    }
}
// end lvgl


static void gfx_initialization(void)
{
    APP_ERROR_CHECK(nrf_gfx_init(p_lcd));
}

int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("GFX usage example application started.")
    NRF_LOG_FLUSH();

    gfx_initialization();
    nrf_delay_ms(10);
    nrf_gfx_rotation_set(p_lcd, NRF_LCD_ROTATE_270);

    lv_init();

    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX / 10);    /*Initialize the display buffer*/

    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

    lv_obj_t *screenMain = lv_obj_create(NULL, NULL);
    label = lv_label_create(screenMain, NULL);
    lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);
    lv_label_set_text(label, "Press a button");
    lv_label_set_align(label, LV_LABEL_ALIGN_LEFT);
    lv_obj_set_size(label, 160, 40);
    lv_obj_set_pos(label, 10, 0);

    UNUSED_VARIABLE(event_handler_btn);

    btn1 = lv_btn_create(screenMain, NULL);
    lv_obj_set_event_cb(btn1, event_handler_btn);
    lv_obj_set_width(btn1, 70);
    lv_obj_set_height(btn1, 32);
    lv_obj_set_pos(btn1, 10, 40);

    btn2 = lv_btn_create(screenMain, NULL);
    lv_obj_set_event_cb(btn2, event_handler_btn);
    lv_obj_set_width(btn2, 70);
    lv_obj_set_height(btn2, 32);
    lv_obj_set_pos(btn2, 100, 40);

    lv_anim_t a;
    lv_anim_init(&a);

    lv_scr_load(screenMain);

    float px = 0;

    while (1)
    {
        px += 0.01;
        if (px > 100) {
            px = 0;
        }

        lv_obj_set_pos(btn2, floor(px), 40);
        lv_obj_set_pos(btn1, floor(100-px), 20);
        lv_tick_inc(1);
        // nrf_delay_ms(1);
        lv_task_handler();
    }
}

