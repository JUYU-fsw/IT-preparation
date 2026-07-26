#include "line_sensor.h"
#include "zf_driver_delay.h"

static uint8 line_sensor_history[LINE_SENSOR_FILTER_SAMPLES];
static uint8 line_sensor_history_count;
static uint8 line_sensor_history_index;
static int16 line_sensor_last_error;

static void line_sensor_select (uint8 channel)
{
    gpio_set_level(LINE_SENSOR_ADDRESS_0_PIN, channel & 0x01);
    gpio_set_level(LINE_SENSOR_ADDRESS_1_PIN, (channel >> 1) & 0x01);
    gpio_set_level(LINE_SENSOR_ADDRESS_2_PIN, (channel >> 2) & 0x01);
}

void line_sensor_init (void)
{
    uint8 index;

    gpio_init(LINE_SENSOR_ADDRESS_0_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_ADDRESS_1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_ADDRESS_2_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_DATA_PIN, GPI, 0, GPI_PULL_DOWN);

    for(index = 0; index < LINE_SENSOR_FILTER_SAMPLES; index ++)
    {
        line_sensor_history[index] = 0;
    }
    line_sensor_history_count = 0;
    line_sensor_history_index = 0;
    line_sensor_last_error = 0;
}

static uint8 line_sensor_count_bits (uint8 mask)
{
    uint8 count = 0;

    while(mask)
    {
        count += mask & 0x01U;
        mask >>= 1;
    }
    return count;
}

static uint8 line_sensor_majority_filter (uint8 raw_mask)
{
    uint8 channel;
    uint8 sample;
    uint8 vote_count;
    uint8 filtered_mask = 0;
    uint8 required_votes;

    line_sensor_history[line_sensor_history_index] = raw_mask;
    line_sensor_history_index ++;
    if(line_sensor_history_index >= LINE_SENSOR_FILTER_SAMPLES)
    {
        line_sensor_history_index = 0;
    }
    if(line_sensor_history_count < LINE_SENSOR_FILTER_SAMPLES)
    {
        line_sensor_history_count ++;
    }

    required_votes = (uint8)(line_sensor_history_count / 2U + 1U);
    for(channel = 0; channel < 8; channel ++)
    {
        vote_count = 0;
        for(sample = 0; sample < line_sensor_history_count; sample ++)
        {
            if(0 != (line_sensor_history[sample] & (uint8)(1U << channel)))
            {
                vote_count ++;
            }
        }
        if(vote_count >= required_votes)
        {
            filtered_mask |= (uint8)(1U << channel);
        }
    }
    return filtered_mask;
}

static uint8 line_sensor_select_segment (uint8 filtered_mask,
                                         uint8 *segment_count)
{
    static const int16 weight[8] =
    {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    uint8 channel;
    uint8 start;
    uint8 length;
    uint8 mask;
    uint8 best_mask = 0;
    uint8 best_length = 0;
    int16 center;
    int16 distance;
    int16 best_distance = 32767;

    *segment_count = 0;
    channel = 0;
    while(channel < 8)
    {
        if(0 == (filtered_mask & (uint8)(1U << channel)))
        {
            channel ++;
            continue;
        }

        start = channel;
        length = 0;
        mask = 0;
        while((channel < 8)
              && (0 != (filtered_mask & (uint8)(1U << channel))))
        {
            mask |= (uint8)(1U << channel);
            length ++;
            channel ++;
        }
        (*segment_count) ++;

        center = (int16)((weight[start]
                 + weight[(uint8)(start + length - 1U)]) / 2);
        distance = center - line_sensor_last_error;
        if(distance < 0)
        {
            distance = (int16)-distance;
        }

        /* Prefer the wider continuous block, then the block nearest history. */
        if((length > best_length)
           || ((length == best_length) && (distance < best_distance)))
        {
            best_length = length;
            best_distance = distance;
            best_mask = mask;
        }
    }
    return best_mask;
}

void line_sensor_read (line_sensor_data_struct *data)
{
    static const int16 weight[8] =
    {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    uint8 channel;
    uint8 active;
    int16 weighted_sum = 0;

    data->raw_mask = 0;

    data->line_valid = 0;
    data->confidence = 0;
    data->state = LINE_SENSOR_STATE_LOST;
    data->error = line_sensor_last_error;

    for(channel = 0; channel < 8; channel ++)
    {
        line_sensor_select(channel);
        system_delay_us(LINE_SENSOR_SETTLE_US);
        active = (gpio_get_level(LINE_SENSOR_DATA_PIN)
                  == LINE_SENSOR_ACTIVE_LEVEL);
        if(active)
        {
            data->raw_mask |= (uint8)(1U << channel);
        }
    }

    data->filtered_mask = line_sensor_majority_filter(data->raw_mask);
    data->active_count = line_sensor_count_bits(data->filtered_mask);
    data->mask = line_sensor_select_segment(data->filtered_mask,
                                            &data->segment_count);

    if(0 == data->filtered_mask)
    {
        return;
    }
    if(0xFFU == data->filtered_mask)
    {
        data->state = LINE_SENSOR_STATE_ALL_BLACK;
        return;
    }

    data->active_count = line_sensor_count_bits(data->mask);
    for(channel = 0; channel < 8; channel ++)
    {
        if(0 != (data->mask & (uint8)(1U << channel)))
        {
            weighted_sum += weight[channel];
        }
    }
    if(0 != data->active_count)
    {
        data->error = weighted_sum / data->active_count;
        data->line_valid = 1;
        line_sensor_last_error = data->error;
    }

    if(data->segment_count > 1U)
    {
        data->state = LINE_SENSOR_STATE_NOISY;
        data->confidence = (data->active_count >= 2U) ? 55U : 35U;
    }
    else if(data->active_count > LINE_SENSOR_NORMAL_MAX_WIDTH)
    {
        data->state = LINE_SENSOR_STATE_WIDE;
        data->confidence = 45U;
    }
    else
    {
        data->state = LINE_SENSOR_STATE_NORMAL;
        data->confidence = (2U == data->active_count) ? 100U : 80U;
    }
}
