#ifndef __CARIBOULITE_CONFIG_DEFAULT_H__
#define __CARIBOULITE_CONFIG_DEFAULT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "cariboulite_config.h"

// PINOUT SPI
#define CARIBOULITE_SPI_DEV 1
#define CARIBOULITE_MOSI 20
#define CARIBOULITE_SCK 21
#define CARIBOULITE_MISO 19

// PINOUT FPGA - ICE40
#define CARIBOULITE_FPGA_SPI_CHANNEL 0
#define CARIBOULITE_FPGA_SS 18
#define CARIBOULITE_FPGA_CDONE 27
#define CARIBOULITE_FPGA_CRESET 26

// PINOUT AT86 - AT86RF215
#define CARIBOULITE_MODEM_SPI_CHANNEL 1
#define CARIBOULITE_MODEM_SS 17
#define CARIBOULITE_MODEM_IRQ 22
#define CARIBOULITE_MODEM_RESET 23

//=======================================================================================
// SYSTEM DEFINITIONS & CONFIGURATIONS
//=======================================================================================
#define CARIBOULITE_CONFIG_DEFAULT(a)                                   \
                cariboulite_st(a)={                                     \
                    .board_info = {0},                                  \
                    .spi_dev =                                          \
                    {                                                   \
                        .miso = CARIBOULITE_MISO,                       \
                        .mosi = CARIBOULITE_MOSI,                       \
                        .sck = CARIBOULITE_SCK,                         \
                        .initialized = 0,                               \
                    },                                                  \
                    .ice40 =                                            \
                    {                                                   \
                        .cs_pin = CARIBOULITE_FPGA_SS,                  \
                        .cdone_pin = CARIBOULITE_FPGA_CDONE,            \
                        .reset_pin = CARIBOULITE_FPGA_CRESET,           \
                        .verbose = 1,                                   \
                        .initialized = 0,                               \
                    },                                                  \
                    .fpga =                                             \
                    {                                                   \
                        .reset_pin = CARIBOULITE_FPGA_CRESET,           \
                        .cs_pin = CARIBOULITE_FPGA_SS,                  \
                        .spi_dev = CARIBOULITE_SPI_DEV,                 \
                        .spi_channel = CARIBOULITE_FPGA_SPI_CHANNEL,    \
                        .initialized = 0,                               \
                    },                                                  \
                    .modem =                                            \
                    {                                                   \
                        .reset_pin = CARIBOULITE_MODEM_RESET,           \
                        .irq_pin = CARIBOULITE_MODEM_IRQ,               \
                        .cs_pin = CARIBOULITE_MODEM_SS,                 \
                        .spi_dev = CARIBOULITE_SPI_DEV,                 \
                        .spi_channel = CARIBOULITE_MODEM_SPI_CHANNEL,   \
                        .initialized = 0,                               \
                        .override_cal = true,                           \
                    },                                                  \
                    .reset_fpga_on_startup = 1,                         \
					.system_status = cariboulite_sys_status_unintialized,\
                }

#ifdef __cplusplus
}
#endif

#endif //__CARIBOULITE_CONFIG_DEFAULT_H__
