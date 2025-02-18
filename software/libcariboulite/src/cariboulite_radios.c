#ifndef ZF_LOG_LEVEL
    #define ZF_LOG_LEVEL ZF_LOG_VERBOSE
#endif
#define ZF_LOG_DEF_SRCLOC ZF_LOG_SRCLOC_LONG
#define ZF_LOG_TAG "CARIBOULITE Radios"
#include "zf_log/zf_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <linux/random.h>
#include <sys/ioctl.h>

#include "cariboulite_radios.h"
#include "cariboulite_setup.h"


#define GET_RADIO_PTR(radio,chan)   ((chan)==cariboulite_channel_s1g?&((radio)->radio_sub1g):&((radio)->radio_6g))
#define GET_CH(rad_ch)              ((rad_ch)==cariboulite_channel_s1g?at86rf215_rf_channel_900mhz:at86rf215_rf_channel_2400mhz)
//======================================================================
typedef struct {
    int bit_count;               /* number of bits of entropy in data */
    int byte_count;              /* number of bytes of data in array */
    unsigned char buf[1];
} entropy_t;

static int add_entropy(uint8_t byte)
{
    int rand_fid = open("/dev/urandom", O_RDWR);
    if (rand_fid != 0)
    {
        // error opening device
        ZF_LOGE("Opening /dev/urandom device file failed");
        return -1;
    }

    entropy_t ent = {
        .bit_count = 8,
        .byte_count = 1,
        .buf = {byte},
    };

    if (ioctl(rand_fid, RNDADDENTROPY, &ent) != 0)
    {
        ZF_LOGE("IOCTL to /dev/urandom device file failed");
    }

    if (close(rand_fid) !=0 )
    {
        ZF_LOGE("Closing /dev/urandom device file failed");
        return -1;
    }

    return 0;
}

//======================================================================
int cariboulite_init_radios(cariboulite_radios_st* radios, cariboulite_st *sys)
{
    memset (radios, 0, sizeof(cariboulite_radios_st));

    // Sub-1GHz
    radios->radio_sub1g.cariboulite_sys = sys;
    radios->radio_sub1g.active = true;
    radios->radio_sub1g.channel_direction = cariboulite_channel_dir_rx;
    radios->radio_sub1g.type = cariboulite_channel_s1g;
    radios->radio_sub1g.cw_output = false;
    radios->radio_sub1g.lo_output = false;
    radios->radio_sub1g.rx_stream_id = -1;
    radios->radio_sub1g.tx_stream_id = -1;

    // Wide band channel
    radios->radio_6g.cariboulite_sys = sys;
    radios->radio_6g.active = true;
    radios->radio_6g.channel_direction = cariboulite_channel_dir_rx;
    radios->radio_6g.type = cariboulite_channel_6g;
    radios->radio_6g.cw_output = false;
    radios->radio_6g.lo_output = false;
    radios->radio_6g.rx_stream_id = -1;
    radios->radio_6g.tx_stream_id = -1;

    cariboulite_sync_radio_information(radios);
}

//======================================================================
int cariboulite_dispose_radios(cariboulite_radios_st* radios)
{
    radios->radio_sub1g.active = false;
    radios->radio_6g.active = false;

#if 0
    // If streams are active - destroy them
    if (radios->radio_sub1g.rx_stream_id != -1)
    {
        caribou_smi_destroy_stream(&radios->radio_sub1g.cariboulite_sys->smi, radios->radio_sub1g.rx_stream_id);
        radios->radio_sub1g.rx_stream_id = -1;
    }

    if (radios->radio_sub1g.tx_stream_id != -1)
    {
        caribou_smi_destroy_stream(&radios->radio_sub1g.cariboulite_sys->smi, radios->radio_sub1g.tx_stream_id);
        radios->radio_sub1g.tx_stream_id = -1;
    }

    if (radios->radio_6g.rx_stream_id != -1)
    {
        caribou_smi_destroy_stream(&radios->radio_6g.cariboulite_sys->smi, radios->radio_sub1g.rx_stream_id);
        radios->radio_6g.rx_stream_id = -1;
    }

    if (radios->radio_6g.tx_stream_id != -1)
    {
        caribou_smi_destroy_stream(&radios->radio_6g.cariboulite_sys->smi, radios->radio_sub1g.tx_stream_id);
        radios->radio_6g.tx_stream_id = -1;
    }
    usleep(100000);
#endif

    cariboulite_radio_state_st* rad_s1g = GET_RADIO_PTR(radios,cariboulite_channel_s1g);
    cariboulite_radio_state_st* rad_6g = GET_RADIO_PTR(radios,cariboulite_channel_6g);

    at86rf215_radio_set_state( &rad_s1g->cariboulite_sys->modem, 
                                    GET_CH(cariboulite_channel_s1g), 
                                    at86rf215_radio_state_cmd_trx_off);
    rad_s1g->state = at86rf215_radio_state_cmd_trx_off;

    at86rf215_radio_set_state( &rad_6g->cariboulite_sys->modem, 
                                    GET_CH(cariboulite_channel_6g), 
                                    at86rf215_radio_state_cmd_trx_off);
    rad_6g->state = at86rf215_radio_state_cmd_trx_off;

    caribou_fpga_set_io_ctrl_mode (&rad_6g->cariboulite_sys->fpga, 0, caribou_fpga_io_ctrl_rfm_low_power);
}

//======================================================================
int cariboulite_sync_radio_information(cariboulite_radios_st* radios)
{
    cariboulite_get_mod_state (radios, cariboulite_channel_s1g, NULL);
    cariboulite_get_mod_state (radios, cariboulite_channel_6g, NULL);

    cariboulite_get_rx_gain_control(radios, cariboulite_channel_s1g, NULL, NULL);
    cariboulite_get_rx_gain_control(radios, cariboulite_channel_6g, NULL, NULL);      

    cariboulite_get_rx_bandwidth(radios, cariboulite_channel_s1g, NULL);
    cariboulite_get_rx_bandwidth(radios, cariboulite_channel_6g, NULL);

    cariboulite_get_tx_power(radios, cariboulite_channel_s1g, NULL);
    cariboulite_get_tx_power(radios, cariboulite_channel_6g, NULL);

    cariboulite_get_rssi(radios, cariboulite_channel_s1g, NULL);
    cariboulite_get_rssi(radios, cariboulite_channel_6g, NULL);

    cariboulite_get_energy_det(radios, cariboulite_channel_s1g, NULL);
    cariboulite_get_energy_det(radios, cariboulite_channel_6g, NULL);
}

//======================================================================
int cariboulite_get_mod_state ( cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel,
                                at86rf215_radio_state_cmd_en *state)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    rad->state  = at86rf215_radio_get_state(&rad->cariboulite_sys->modem, GET_CH(channel));

    if (state) *state = rad->state;
    return 0;
}


//======================================================================
int cariboulite_get_mod_intertupts (cariboulite_radios_st* radios, 
                                    cariboulite_channel_en channel,
                                    at86rf215_radio_irq_st **irq_table)
{
    at86rf215_irq_st irq = {0};
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_get_irqs(&rad->cariboulite_sys->modem, &irq, 0);

    if (channel == cariboulite_channel_s1g)
    {
        memcpy (&rad->interrupts, &irq.radio09, sizeof(at86rf215_radio_irq_st));
        if (irq_table) *irq_table = &irq.radio09;
    }
    else
    {
        memcpy (&rad->interrupts, &irq.radio24, sizeof(at86rf215_radio_irq_st));
        if (irq_table) *irq_table = &irq.radio24;
    }

    return 0;
}

//======================================================================
int cariboulite_set_rx_gain_control(cariboulite_radios_st* radios, 
                                    cariboulite_channel_en channel,
                                    bool rx_agc_on,
                                    int rx_gain_value_db)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);

    int control_gain_val = (int)roundf((float)(rx_gain_value_db) / 3.0f);
    if (control_gain_val < 0) control_gain_val = 0;
    if (control_gain_val > 23) control_gain_val = 23;

    at86rf215_radio_agc_ctrl_st rx_gain_control = 
    {
        .agc_measure_source_not_filtered = 1,
        .avg = at86rf215_radio_agc_averaging_32,
        .reset_cmd = 0,
        .freeze_cmd = 0,
        .enable_cmd = rx_agc_on,
        .att = at86rf215_radio_agc_relative_atten_21_db,
        .gain_control_word = control_gain_val,
    };

    at86rf215_radio_setup_agc(&rad->cariboulite_sys->modem, GET_CH(channel), &rx_gain_control);
    rad->rx_agc_on = rx_agc_on;
    rad->rx_gain_value_db = rx_gain_value_db;
    return 0;
}

//======================================================================
int cariboulite_get_rx_gain_control(cariboulite_radios_st* radios, 
                                    cariboulite_channel_en channel,
                                    bool *rx_agc_on,
                                    int *rx_gain_value_db)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_agc_ctrl_st agc_ctrl = {0};
    at86rf215_radio_get_agc(&rad->cariboulite_sys->modem, GET_CH(channel), &agc_ctrl);

    rad->rx_agc_on = agc_ctrl.enable_cmd;
    rad->rx_gain_value_db = agc_ctrl.gain_control_word * 3;
    if (rx_agc_on) *rx_agc_on = rad->rx_agc_on;
    if (rx_gain_value_db) *rx_gain_value_db = rad->rx_gain_value_db;
    return 0;
}

//======================================================================
int cariboulite_get_rx_gain_limits(cariboulite_radios_st* radios, 
                                    cariboulite_channel_en channel,
                                    int *rx_min_gain_value_db,
                                    int *rx_max_gain_value_db,
                                    int *rx_gain_value_resolution_db)
{
    if (rx_min_gain_value_db) *rx_min_gain_value_db = 0;
    if (rx_max_gain_value_db) *rx_max_gain_value_db = 23*3;
    if (rx_gain_value_resolution_db) *rx_gain_value_resolution_db = 3;
    return 0;
}

//======================================================================
int cariboulite_set_rx_bandwidth(cariboulite_radios_st* radios, 
                                 cariboulite_channel_en channel,
                                 at86rf215_radio_rx_bw_en rx_bw)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_f_cut_en fcut = at86rf215_radio_rx_f_cut_half_fs;

    // Automatically calculate the digital f_cut
    if (rx_bw >= at86rf215_radio_rx_bw_BW160KHZ_IF250KHZ && rx_bw <= at86rf215_radio_rx_bw_BW500KHZ_IF500KHZ)
        fcut = at86rf215_radio_rx_f_cut_0_25_half_fs;
    else if (rx_bw >= at86rf215_radio_rx_bw_BW630KHZ_IF1000KHZ && rx_bw <= at86rf215_radio_rx_bw_BW630KHZ_IF1000KHZ)
        fcut = at86rf215_radio_rx_f_cut_0_375_half_fs;
    else if (rx_bw >= at86rf215_radio_rx_bw_BW800KHZ_IF1000KHZ && rx_bw <= at86rf215_radio_rx_bw_BW1000KHZ_IF1000KHZ)
        fcut = at86rf215_radio_rx_f_cut_0_5_half_fs;
    else if (rx_bw >= at86rf215_radio_rx_bw_BW1250KHZ_IF2000KHZ && rx_bw <= at86rf215_radio_rx_bw_BW1250KHZ_IF2000KHZ)
        fcut = at86rf215_radio_rx_f_cut_0_75_half_fs;
    else 
        fcut = at86rf215_radio_rx_f_cut_half_fs;

    rad->rx_fcut = fcut;

    at86rf215_radio_set_rx_bw_samp_st cfg = 
    {
        .inverter_sign_if = 0,
        .shift_if_freq = 1,                 // A value of one configures the receiver to shift the IF frequency
                                            // by factor of 1.25. This is useful to place the image frequency according
                                            // to channel scheme. This increases the IF frequency to max 2.5MHz
                                            // thus places the internal LO fasr away from the signal => lower noise
        .bw = rx_bw,
        .fcut = rad->rx_fcut,               // keep the same
        .fs = rad->rx_fs,                   // keep the same
    };
    at86rf215_radio_set_rx_bandwidth_sampling(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->rx_bw = rx_bw;
    return 0;
}

//======================================================================
int cariboulite_get_rx_bandwidth(cariboulite_radios_st* radios, 
                                 cariboulite_channel_en channel,
                                 at86rf215_radio_rx_bw_en *rx_bw)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_set_rx_bw_samp_st cfg = {0};
    at86rf215_radio_get_rx_bandwidth_sampling(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->rx_bw = cfg.bw;
    rad->rx_fcut = cfg.fcut;
    rad->rx_fs = cfg.fs;
    if (rx_bw) *rx_bw = rad->rx_bw;
    return 0;
}

//======================================================================
int cariboulite_set_rx_samp_cutoff(cariboulite_radios_st* radios, 
                                   cariboulite_channel_en channel,
                                   at86rf215_radio_sample_rate_en rx_sample_rate,
                                   at86rf215_radio_f_cut_en rx_cutoff)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);

    at86rf215_radio_set_rx_bw_samp_st cfg = 
    {
        .inverter_sign_if = 0,              // A value of one configures the receiver to implement the inverted-sign
                                            // IF frequency. Use default setting for normal operation
        .shift_if_freq = 1,                 // A value of one configures the receiver to shift the IF frequency
                                            // by factor of 1.25. This is useful to place the image frequency according
                                            // to channel scheme.
        .bw = rad->rx_bw,                   // keep the same
        .fcut = rx_cutoff,
        .fs = rx_sample_rate,
    };
    at86rf215_radio_set_rx_bandwidth_sampling(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->rx_fs = rx_sample_rate;
    rad->rx_fcut = rx_cutoff;
    return 0;
}

//======================================================================
int cariboulite_get_rx_samp_cutoff(cariboulite_radios_st* radios, 
                                   cariboulite_channel_en channel,
                                   at86rf215_radio_sample_rate_en *rx_sample_rate,
                                   at86rf215_radio_f_cut_en *rx_cutoff)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    cariboulite_get_rx_bandwidth(radios, channel, NULL);
    if (rx_sample_rate) *rx_sample_rate = rad->rx_fs;
    if (rx_cutoff) *rx_cutoff = rad->rx_fcut;
    return 0;
}

//======================================================================
int cariboulite_set_tx_power(cariboulite_radios_st* radios, 
                             cariboulite_channel_en channel,
                             int tx_power_dbm)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    if (tx_power_dbm < -18) tx_power_dbm = -18;
    if (tx_power_dbm > 13) tx_power_dbm = 13;
    int tx_power_ctrl = 18 + tx_power_dbm;

    at86rf215_radio_tx_ctrl_st cfg = 
    {
        .pa_ramping_time = at86rf215_radio_tx_pa_ramp_16usec,
        .current_reduction = at86rf215_radio_pa_current_reduction_0ma,  // we can use this to gain some more
                                                                        // granularity with the tx gain control
        .tx_power = tx_power_ctrl,
        .analog_bw = rad->tx_bw,            // same as before
        .digital_bw = rad->tx_fcut,         // same as before
        .fs = rad->tx_fs,                   // same as before
        .direct_modulation = 0,
    };

    at86rf215_radio_setup_tx_ctrl(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->tx_power = tx_power_dbm;

    return 0;
}

//======================================================================
int cariboulite_get_tx_power(cariboulite_radios_st* radios, 
                             cariboulite_channel_en channel,
                             int *tx_power_dbm)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_tx_ctrl_st cfg = {0};
    at86rf215_radio_get_tx_ctrl(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->tx_power = cfg.tx_power - 18;
    rad->tx_bw = cfg.analog_bw;
    rad->tx_fcut = cfg.digital_bw;
    rad->tx_fs = cfg.fs;

    if (tx_power_dbm) *tx_power_dbm = rad->tx_power;
    return 0;
}

//======================================================================
int cariboulite_set_tx_bandwidth(cariboulite_radios_st* radios, 
                                 cariboulite_channel_en channel,
                                 at86rf215_radio_tx_cut_off_en tx_bw)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_tx_ctrl_st cfg = 
    {
        .pa_ramping_time = at86rf215_radio_tx_pa_ramp_16usec,
        .current_reduction = at86rf215_radio_pa_current_reduction_0ma,  // we can use this to gain some more
                                                                        // granularity with the tx gain control
        .tx_power = 18 + rad->tx_power,     // same as before
        .analog_bw = tx_bw,
        .digital_bw = rad->tx_fcut,         // same as before
        .fs = rad->tx_fs,                   // same as before
        .direct_modulation = 0,
    };

    at86rf215_radio_setup_tx_ctrl(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->tx_bw = tx_bw;

    return 0;
}

//======================================================================
int cariboulite_get_tx_bandwidth(cariboulite_radios_st* radios, 
                                 cariboulite_channel_en channel,
                                 at86rf215_radio_tx_cut_off_en *tx_bw)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    cariboulite_get_tx_power(radios, channel, NULL);
    if (tx_bw) *tx_bw = rad->tx_bw;
    return 0;
}

//======================================================================
int cariboulite_set_tx_samp_cutoff(cariboulite_radios_st* radios, 
                                   cariboulite_channel_en channel,
                                   at86rf215_radio_sample_rate_en tx_sample_rate,
                                   at86rf215_radio_f_cut_en tx_cutoff)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_tx_ctrl_st cfg = 
    {
        .pa_ramping_time = at86rf215_radio_tx_pa_ramp_16usec,
        .current_reduction = at86rf215_radio_pa_current_reduction_0ma,  // we can use this to gain some more
                                                                        // granularity with the tx gain control
        .tx_power = 18 + rad->tx_power,     // same as before
        .analog_bw = rad->tx_bw,            // same as before
        .digital_bw = tx_cutoff,
        .fs = tx_sample_rate,
        .direct_modulation = 0,
    };

    at86rf215_radio_setup_tx_ctrl(&rad->cariboulite_sys->modem, GET_CH(channel), &cfg);
    rad->tx_fcut = tx_cutoff;
    rad->tx_fs = tx_sample_rate;
    
    return 0;
}

//======================================================================
int cariboulite_get_tx_samp_cutoff(cariboulite_radios_st* radios, 
                                   cariboulite_channel_en channel,
                                   at86rf215_radio_sample_rate_en *tx_sample_rate,
                                   at86rf215_radio_f_cut_en *rx_cutoff)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    cariboulite_get_tx_power(radios, channel, NULL);
    if (tx_sample_rate) *tx_sample_rate = rad->tx_fs;
    if (rx_cutoff) *rx_cutoff = rad->tx_fcut;
    return 0;
}

//======================================================================
int cariboulite_get_rssi(cariboulite_radios_st* radios, cariboulite_channel_en channel, float *rssi_dbm)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    float rssi = at86rf215_radio_get_rssi_dbm(&rad->cariboulite_sys->modem, GET_CH(channel));
    if (rssi >= -127.0 && rssi <= 4)   // register only valid values
    {
        rad->rx_rssi = rssi;
        if (rssi_dbm) *rssi_dbm = rssi;
        return 0;
    }

    if (rssi_dbm) *rssi_dbm = rad->rx_rssi;
    return -1;
}

//======================================================================
int cariboulite_get_energy_det(cariboulite_radios_st* radios, cariboulite_channel_en channel, float *energy_det_val)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    at86rf215_radio_energy_detection_st det = {0};
    at86rf215_radio_get_energy_detection(&rad->cariboulite_sys->modem, GET_CH(channel), &det);
    
    if (det.energy_detection_value >= -127.0 && det.energy_detection_value <= 4)   // register only valid values
    {
        rad->rx_energy_detection_value = det.energy_detection_value;
        if (energy_det_val) *energy_det_val = rad->rx_energy_detection_value;
        return 0;
    }

    if (energy_det_val) *energy_det_val = rad->rx_energy_detection_value;
    return -1;
}

//======================================================================
int cariboulite_get_rand_val(cariboulite_radios_st* radios, cariboulite_channel_en channel, uint8_t *rnd)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    rad->random_value = at86rf215_radio_get_random_value(&rad->cariboulite_sys->modem, GET_CH(channel));
    if (rnd) *rnd = rad->random_value;
    add_entropy(rad->random_value);
    return 0;
}


//=================================================
#define CARIBOULITE_MIN_MIX     (1.0e6)        // 30
#define CARIBOULITE_MAX_MIX     (6000.0e6)      // 6000
#define CARIBOULITE_MIN_LO      (85.0e6)
#define CARIBOULITE_MAX_LO      (4200.0e6)
#define CARIBOULITE_2G4_MIN     (2395.0e6)      // 2400
#define CARIBOULITE_2G4_MAX     (2485.0e6)      // 2483.5
//#define CARIBOULITE_S1G_MIN1    (389.5e6)
#define CARIBOULITE_S1G_MIN1    (350.0e6)
#define CARIBOULITE_S1G_MAX1    (510.0e6)
#define CARIBOULITE_S1G_MIN2    (779.0e6)
#define CARIBOULITE_S1G_MAX2    (1020.0e6)

typedef enum
{
    conversion_dir_none = 0,
    conversion_dir_up = 1,
    conversion_dir_down = 2,
} conversion_dir_en;

//=================================================
bool cariboulite_wait_for_lock( cariboulite_radio_state_st* rad, bool *mod, bool *mix, int retries)
{
    bool mod_lock = true;

    if (mod)
    {
        at86rf215_radio_pll_ctrl_st cfg = {0};
        int relock_retries = retries;
        do
        {
            at86rf215_rf_channel_en ch = rad->type == cariboulite_channel_s1g ? 
                        at86rf215_rf_channel_900mhz : at86rf215_rf_channel_2400mhz;
            at86rf215_radio_get_pll_ctrl(&rad->cariboulite_sys->modem, ch, &cfg);
        } while (!cfg.pll_locked && relock_retries--);

        *mod = cfg.pll_locked;
        mod_lock = (bool)cfg.pll_locked;
    }

    return mod_lock;
}

//=================================================
int cariboulite_set_frequency(  cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel, 
                                bool break_before_make,
                                double *freq)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);

    double f_rf = *freq;
    double modem_act_freq = 0.0;
    double lo_act_freq = 0.0;
    double act_freq = 0.0;
    int error = 0;

    //--------------------------------------------------------------------------------
    // SUB 1GHZ CONFIGURATION
    //--------------------------------------------------------------------------------
    if (channel == cariboulite_channel_s1g)
    {
        if ( (f_rf >= CARIBOULITE_S1G_MIN1 && f_rf <= CARIBOULITE_S1G_MAX1) ||
             (f_rf >= CARIBOULITE_S1G_MIN2 && f_rf <= CARIBOULITE_S1G_MAX2)   )
        {
            // setup modem frequency <= f_rf
            if (break_before_make)
            {
                at86rf215_radio_set_state(  &rad->cariboulite_sys->modem, 
                                        at86rf215_rf_channel_900mhz, 
                                        at86rf215_radio_state_cmd_trx_off);
                rad->state = at86rf215_radio_state_cmd_trx_off;
            }

            modem_act_freq = at86rf215_setup_channel (&rad->cariboulite_sys->modem, 
                                                    at86rf215_rf_channel_900mhz, 
                                                    (uint32_t)f_rf);

            at86rf215_radio_pll_ctrl_st cfg = {0};
            at86rf215_radio_get_pll_ctrl(&rad->cariboulite_sys->modem, at86rf215_rf_channel_900mhz, &cfg);

            rad->if_frequency = 0;
            rad->lo_pll_locked = 1;
            rad->modem_pll_locked = cfg.pll_locked;
            rad->if_frequency = modem_act_freq;
            rad->actual_rf_frequency = rad->if_frequency;
            rad->requested_rf_frequency = f_rf;
            rad->rf_frequency_error = rad->actual_rf_frequency - rad->requested_rf_frequency;   

            // return actual frequency
            *freq = rad->actual_rf_frequency;
        }
        else
        {
            // error - unsupported frequency for S1G channel
            ZF_LOGE("unsupported frequency for S1G channel - %.2f Hz", f_rf);
            error = -1;
        }
    }

    if (error >= 0)
    {
        ZF_LOGD("Frequency setting CH: %d, Wanted: %.2f Hz, Set: %.2f Hz (MOD: %.2f, MIX: %.2f)", 
                        channel, f_rf, act_freq, modem_act_freq, lo_act_freq);
    }

    return -error;
}

//======================================================================
int cariboulite_get_frequency(  cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel, 
                                double *freq, double *lo, double* i_f)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    if (freq) *freq = rad->actual_rf_frequency;
    if (lo) *lo = rad->lo_frequency;
    if (i_f) *i_f = rad->if_frequency;
}

//======================================================================
int cariboulite_setup_stream(cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel,
                                cariboulite_channel_dir_en channel_dir
                                ) {

    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    rad->channel_direction = channel_dir;
    return 0;

}
int cariboulite_activate_channel(cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel,
                                bool active)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);

    /* FIXME patch */
    ZF_LOGD("Activating channel %d, dir = %s, active = %d", channel, rad->channel_direction==cariboulite_channel_dir_rx?"RX":"TX", active);
    // if the channel state is active, turn it off before reactivating
    if (rad->state != at86rf215_radio_state_cmd_tx_prep)
    {
        at86rf215_radio_set_state( &rad->cariboulite_sys->modem, 
                                    GET_CH(channel), 
                                    at86rf215_radio_state_cmd_tx_prep);
        rad->state = at86rf215_radio_state_cmd_tx_prep;
        ZF_LOGD("Setup Modem state tx_prep");
    }

    if (!active)
    {
        at86rf215_radio_set_state( &rad->cariboulite_sys->modem, 
                                    GET_CH(channel), 
                                    at86rf215_radio_state_cmd_trx_off);
        rad->state = at86rf215_radio_state_cmd_trx_off;
        ZF_LOGD("Setup Modem state trx_off");
        return 0;
    }

    // Activate the channel according to the configurations
    // RX on both channels looks the same
    if (rad->channel_direction == cariboulite_channel_dir_rx)
    {
        at86rf215_radio_set_state( &rad->cariboulite_sys->modem, 
                                GET_CH(channel),
                                at86rf215_radio_state_cmd_rx);
        rad->state = at86rf215_radio_state_cmd_rx;
        ZF_LOGD("Setup Modem state cmd_rx");
		caribou_fpga_set_trx_state_rx (&rad->cariboulite_sys->fpga, 0, caribou_fpga_io_ctrl_rfm_low_power);
    }
    else if (rad->channel_direction == cariboulite_channel_dir_tx)
    {
		caribou_fpga_set_trx_state_tx (&rad->cariboulite_sys->fpga, 0, caribou_fpga_io_ctrl_rfm_low_power);
        {
#if 0

			at86rf215_radio_tx_ctrl_st tx_config =
    {
        .pa_ramping_time = at86rf215_radio_tx_pa_ramp_32usec,
        .current_reduction = at86rf215_radio_pa_current_reduction_0ma,
        .tx_power = 10, /* ICI */
        .analog_bw = at86rf215_radio_tx_cut_off_80khz,
        .digital_bw = at86rf215_radio_rx_f_cut_half_fs,
        .fs = at86rf215_radio_rx_sample_rate_4000khz,
        .direct_modulation = 0,
    };
    at86rf215_radio_setup_tx_ctrl(&rad->cariboulite_sys->modem, GET_CH(channel), &tx_config);

#endif
	    at86rf215_setup_channel (&rad->cariboulite_sys->modem, at86rf215_rf_channel_900mhz, rad->requested_rf_frequency);

            // transition to state TX
            //at86rf215_radio_set_state(&rad->cariboulite_sys->modem, 
            //                            GET_CH(channel),
            //                            at86rf215_radio_state_cmd_tx);

            at86rf215_iq_interface_config_st cfg;
	    at86rf215_radio_tx_ctrl_st cfg2;

	    for (int i=0; i<1; i++) {
		    at86rf215_radio_get_tx_ctrl(&rad->cariboulite_sys->modem, at86rf215_rf_channel_900mhz,
                                &cfg2);

		    at86rf215_get_iq_if_cfg(&rad->cariboulite_sys->modem, 
                                        &cfg,
                                        true);
	    }

        }
    }

    return 0;
}

int cariboulite_deactivate_channel(cariboulite_radios_st* radios, 
                                cariboulite_channel_en channel,
								bool active) {
	return 0;
}

//======================================================================
int cariboulite_set_cw_outputs(cariboulite_radios_st* radios, 
                               cariboulite_channel_en channel, bool lo_out, bool cw_out)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);

    if (rad->lo_output && channel == cariboulite_channel_6g)
    {
        rad->lo_output = lo_out;
    }
    else
    {
        rad->lo_output = false;
    }
    rad->cw_output = cw_out;

    if (cw_out)
    {
        rad->channel_direction = cariboulite_channel_dir_tx;
    }
    else
    {
        rad->channel_direction = cariboulite_channel_dir_rx;
    }

    return 0;
}

//======================================================================
int cariboulite_get_cw_outputs(cariboulite_radios_st* radios, 
                               cariboulite_channel_en channel, bool *lo_out, bool *cw_out)
{
    cariboulite_radio_state_st* rad = GET_RADIO_PTR(radios,channel);
    if (lo_out) *lo_out = rad->lo_output;
    if (cw_out) *cw_out = rad->cw_output;

    return 0;
}
