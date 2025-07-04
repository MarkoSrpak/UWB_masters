/*! ----------------------------------------------------------------------------
 * @file    deca_probe_interface.c
 * @brief   Interface structure. Provides external dependencies required by the driver
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */
#include "deca_probe_interface.h"
#include "deca_interface.h"
#include "deca_spi.h"
#include "port.h"

extern const struct dwt_driver_s dw3000_driver;
extern const struct dwt_driver_s dw3720_driver;
const struct dwt_driver_s* tmp_ptr[] = { &dw3000_driver, &dw3720_driver };

static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = readfromspi,
    .writetospi = writetospi,
    .writetospiwithcrc = writetospiwithcrc,
    .setslowrate = port_set_dw_ic_spi_slowrate,
    .setfastrate = port_set_dw_ic_spi_fastrate
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = wakeup_device_with_io,
    .driver_list = (struct dwt_driver_s **)tmp_ptr,
    .dw_driver_num = 2,
};
