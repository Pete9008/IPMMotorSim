/*
 * This file is part of the IPMMotorSim project
 *
 * Copyright (C) 2022 Pete9008 <openinverter.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
extern volatile bool disablePWM;
volatile bool disablePWM = true;

void timer_enable_break_main_output(uint32_t timer_peripheral) {disablePWM = false;(void)timer_peripheral;}
void timer_disable_break_main_output(uint32_t timer_peripheral) {disablePWM = true;(void)timer_peripheral;}

void timer_enable_oc_output(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_disable_irq(uint32_t timer_peripheral, uint32_t irq) {(void)timer_peripheral;(void)irq;}
uint32_t timer_get_counter(uint32_t timer_peripheral) {(void)timer_peripheral;return 0;}
void timer_clear_flag(uint32_t timer_peripheral, uint32_t flag) {(void)timer_peripheral;(void)flag;}
void timer_disable_oc_output(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_set_oc_polarity_low(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_set_oc_polarity_high(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_set_oc_value(uint32_t timer_peripheral, enum tim_oc_id oc_id, uint32_t value) {(void)timer_peripheral;(void)oc_id;(void)value;}
void timer_set_period(uint32_t timer_peripheral, uint32_t period) {(void)timer_peripheral;(void)period;}
void timer_set_alignment(uint32_t timer_peripheral, uint32_t alignment) {(void)timer_peripheral;(void)alignment;}
void timer_enable_preload(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_enable_oc_preload(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_set_oc_mode(uint32_t timer_peripheral, enum tim_oc_id oc_id, enum tim_oc_mode oc_mode) {(void)timer_peripheral;(void)oc_id;(void)oc_mode;}
void timer_set_oc_idle_state_unset(uint32_t timer_peripheral, enum tim_oc_id oc_id) {(void)timer_peripheral;(void)oc_id;}
void timer_set_break_polarity_high(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_set_break_polarity_low(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_enable_break(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_set_enabled_off_state_in_run_mode(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_set_enabled_off_state_in_idle_mode(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_disable_break_automatic_output(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_set_deadtime(uint32_t timer_peripheral, uint32_t deadtime) {(void)timer_peripheral;(void)deadtime;}
void timer_enable_irq(uint32_t timer_peripheral, uint32_t irq) {(void)timer_peripheral;(void)irq;}
void timer_set_prescaler(uint32_t timer_peripheral, uint32_t value) {(void)timer_peripheral;(void)value;}
void timer_set_repetition_counter(uint32_t timer_peripheral, uint32_t value) {(void)timer_peripheral;(void)value;}
void timer_generate_event(uint32_t timer_peripheral, uint32_t event) {(void)timer_peripheral;(void)event;}
void timer_enable_counter(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_disable_counter(uint32_t timer_peripheral) {(void)timer_peripheral;}
void timer_set_clock_division(uint32_t timer_peripheral, uint32_t clock_div) {(void)timer_peripheral;(void)clock_div;}

uint16_t gpio_get(uint32_t gpioport, uint16_t gpios) {(void)gpioport;(void)gpios;return 0;}
void gpio_set(uint32_t gpioport, uint16_t gpios) {(void)gpioport;(void)gpios;}
void gpio_set_mode(uint32_t gpioport, uint8_t mode, uint8_t cnf, uint16_t gpios) {(void)gpioport;(void)mode;(void)cnf;(void)gpios;}

void rcc_periph_reset_pulse(enum rcc_periph_rst rst) {(void)rst;}
