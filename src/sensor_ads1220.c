// Support for ADS1220 ADC Chip
//
// Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "board/gpio.h" // gpio_out_write
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer

struct ads1220_adc {
    struct timer timer;
    uint32_t rest_ticks;
    struct gpio_in data_ready;
    struct spidev_s *spi;
    uint8_t pending_flag, data_count;
    struct sensor_bulk sb;
};

// Flag types
enum {
    FLAG_PENDING = 1 << 0
};

#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR 0x80000000
#define ERROR_READ_TOO_LONG 1
#define ERROR_OUT_OF_RANGE 2

static struct task_wake wake_ads1220;

/****************************************************************
 * ADS1220 Sensor Support
 ****************************************************************/

// Event handler that wakes wake_ads1220() periodically
static uint_fast8_t
ads1220_event(struct timer *timer)
{
    struct ads1220_adc *ads1220 = container_of(timer, struct ads1220_adc,
                                                timer);
    ads1220->pending_flag = 1;
    sched_wake_task(&wake_ads1220);
    return SF_DONE;
}

// Helper code to reschedule the ads1220_event() timer
static void
ads1220_reschedule_timer(struct ads1220_adc *ads1220)
{
    irq_disable();
    ads1220->timer.waketime = timer_read_time() + ads1220->rest_ticks;
    sched_add_timer(&ads1220->timer);
    irq_enable();
}

int8_t
ads1220_is_data_ready(struct ads1220_adc *ads1220) {
    return gpio_in_read(ads1220->data_ready) == 0;
}

// Add a measurement to the buffer
static void
add_sample(struct ads1220_adc *ads1220, uint_fast32_t counts)
{
    ads1220->sb.data[ads1220->sb.data_count] = counts;
    ads1220->sb.data[ads1220->sb.data_count + 1] = counts >> 8;
    ads1220->sb.data[ads1220->sb.data_count + 2] = counts >> 16;
    ads1220->sb.data[ads1220->sb.data_count + 3] = counts >> 24;
    ads1220->sb.data_count += BYTES_PER_SAMPLE;
}

static void
flush_samples(struct ads1220_adc *ads1220, uint8_t oid)
{
    if ((ads1220->sb.data_count + BYTES_PER_SAMPLE) >
            ARRAY_SIZE(ads1220->sb.data)) {
        sensor_bulk_report(&ads1220->sb, oid);
    }
}

// log the error to the data stream and flush
static void
send_error(struct ads1220_adc *ads1220, uint8_t oid, uint8_t error_code)
{
    ads1220->pending_flag = 0;
    // write error codes and send to host
    add_sample(ads1220, SAMPLE_ERROR);
    ads1220->sb.possible_overflows = error_code;
    sensor_bulk_report(&ads1220->sb, oid);
}

// ADS1220 ADC query
void
ads1220_read_adc(struct ads1220_adc *ads1220, uint8_t oid)
{
    if (!ads1220_is_data_ready(ads1220)) {
        ads1220_reschedule_timer(ads1220);
        return;
    }

    // data is ready
    uint8_t msg[3] = {0, 0, 0};
    uint32_t start_time = timer_read_time();
    spidev_transfer(ads1220->spi, 1, sizeof(msg), msg);
    uint32_t time_diff = timer_read_time() - start_time;

    if (time_diff >= ads1220->rest_ticks >> 1) {
        // some IRQ delayed this read so much that its unusable
        send_error(ads1220, oid, ERROR_READ_TOO_LONG);
        return;
    }

    // create 24 bit int from bytes
    int32_t counts = (msg[0] << 16) | (msg[1] << 8) | msg[2];

    // handle -0 value
    if (counts == 0x800000) {
        counts = 0;
    }
    else if (counts > 0x800000) {
        // extend 2's complement 24 bits to 32bits
        counts |= 0xFF000000;
    }

    if (counts < -0x7FFFFF || counts > 0x7FFFFF) {
        send_error(ads1220, oid, ERROR_OUT_OF_RANGE);
        return;
    }

    add_sample(ads1220, counts);
    flush_samples(ads1220, oid);
    ads1220->pending_flag = 0;
    ads1220_reschedule_timer(ads1220);
}

// Create an ads1220 sensor
void
command_config_ads1220(uint32_t *args)
{
    struct ads1220_adc *ads1220 = oid_alloc(args[0]
                , command_config_ads1220, sizeof(*ads1220));
    ads1220->timer.func = ads1220_event;
    ads1220->pending_flag = 0;
    ads1220->spi = spidev_oid_lookup(args[1]);
    ads1220->data_ready = gpio_in_setup(args[2], 0);
}
DECL_COMMAND(command_config_ads1220, "config_ads1220 oid=%c"
    " spi_oid=%c data_ready_pin=%u");

// start/stop capturing ADC data
void
command_query_ads1220(uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads1220_adc *ads1220 = oid_lookup(oid, command_config_ads1220);
    sched_del_timer(&ads1220->timer);
    ads1220->pending_flag = 0;
    ads1220->rest_ticks = args[1];
    if (!ads1220->rest_ticks) {
        // End measurements
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&ads1220->sb);
    ads1220_reschedule_timer(ads1220);
}
DECL_COMMAND(command_query_ads1220, "query_ads1220 oid=%c rest_ticks=%u");

void
command_query_ads1220_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct ads1220_adc *ads1220 = oid_lookup(oid, command_config_ads1220);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_data_ready = ads1220_is_data_ready(ads1220);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? BYTES_PER_SAMPLE : 0;
    sensor_bulk_status(&ads1220->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_ads1220_status, "query_ads1220_status oid=%c");

// Background task that performs measurements
void
ads1220_capture_task(void)
{
    if (!sched_check_wake(&wake_ads1220))
        return;
    uint8_t oid;
    struct ads1220_adc *ads1220;
    foreach_oid(oid, ads1220, command_config_ads1220) {
        if (ads1220->pending_flag) {
            ads1220_read_adc(ads1220, oid);
        }
    }
}
DECL_TASK(ads1220_capture_task);
