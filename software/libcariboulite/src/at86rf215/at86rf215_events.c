#ifndef ZF_LOG_LEVEL
    #define ZF_LOG_LEVEL ZF_LOG_VERBOSE
#endif

#define ZF_LOG_DEF_SRCLOC ZF_LOG_SRCLOC_LONG
#define ZF_LOG_TAG "AT86RF215_Events"

#include <stdio.h>
#include "zf_log/zf_log.h"
#include "at86rf215_common.h"
#include <pthread.h>


void event_node_init(event_st* ev)
{
    pthread_cond_init(&ev->ready_cond, NULL);
    pthread_mutex_init(&ev->ready_mutex, NULL);
}

void event_node_close(event_st* ev)
{
    pthread_cond_destroy(&ev->ready_cond);
    pthread_mutex_destroy(&ev->ready_mutex);
}

void event_node_wait_ready(event_st* ev)
{
    pthread_mutex_lock(&ev->ready_mutex);
    while (!ev->ready)
    {
        pthread_cond_wait(&ev->ready_cond, &ev->ready_mutex);
    }
    ev->ready = 0;
    pthread_mutex_unlock(&ev->ready_mutex);
}

void event_node_signal_ready(event_st* ev, int ready)
{
    pthread_mutex_lock(&ev->ready_mutex);
    ev->ready = ready;
    pthread_cond_signal(&ev->ready_cond);
    pthread_mutex_unlock(&ev->ready_mutex);
}

//===================================================================
static void at86rf215_radio_event_handler (at86rf215_st* dev,
                                at86rf215_rf_channel_en ch,
                                at86rf215_radio_irq_st *events)
{
    char channel_st[3];
    if (ch == at86rf215_rf_channel_900mhz) strcpy(channel_st, "09");
    else strcpy(channel_st, "24");

    if (events->wake_up_por)
    {
        ZF_LOGD("INT @ RADIO%s: Woke up", channel_st);
    }

    if (events->trx_ready)
    {
        ZF_LOGD("INT @ RADIO%s: Transceiver ready", channel_st);
        if (ch == at86rf215_rf_channel_900mhz) event_node_signal_ready(&dev->events.lo_trx_ready_event, 1);
        else if (ch == at86rf215_rf_channel_2400mhz) event_node_signal_ready(&dev->events.hi_trx_ready_event, 1);
    }

    if (events->energy_detection_complete)
    {
        ZF_LOGD("INT @ RADIO%s: Energy detection complete", channel_st);
        if (ch == at86rf215_rf_channel_900mhz) event_node_signal_ready(&dev->events.lo_energy_measure_event, 1);
        else if (ch == at86rf215_rf_channel_2400mhz) event_node_signal_ready(&dev->events.hi_energy_measure_event, 1);
    }

    if (events->battery_low)
    {
        ZF_LOGD("INT @ RADIO%s: Battery low", channel_st);
    }

    if (events->trx_error)
    {
        ZF_LOGD("INT @ RADIO%s: Transceiver error", channel_st);
    }

    if (events->IQ_if_sync_fail)
    {
        ZF_LOGD("INT @ RADIO%s: I/Q interface sync failed", channel_st);
    }
}

//===================================================================
void at86rf215_interrupt_handler (int event, int level, uint32_t tick, void *data)
{
    int i;
    at86rf215_st* dev = (at86rf215_st*)data;
    at86rf215_irq_st irq = {0};

    if (level == 0) return;
    // first read the irqs
    at86rf215_get_irqs(dev, &irq, 0);
    uint8_t *tmp = (uint8_t *)&irq;

    if (tmp[0] != 0) at86rf215_radio_event_handler (dev, at86rf215_rf_channel_900mhz, &irq.radio09);
    if (tmp[1] != 0) at86rf215_radio_event_handler (dev, at86rf215_rf_channel_2400mhz, &irq.radio24);
}
