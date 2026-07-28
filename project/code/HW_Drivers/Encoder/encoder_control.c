#include "encoder_control.h"

encoder_data_t encoder_left = {
    .index = ENCODER_LEFT_INDEX,
    .ch1_pin = ENCODER_LEFT_A,
    .ch2_pin = ENCODER_LEFT_B,
    .count_raw = 0,
    .invert = 0
};

encoder_data_t encoder_right = {
    .index = ENCODER_RIGHT_INDEX,
    .ch1_pin = ENCODER_RIGHT_A,
    .ch2_pin = ENCODER_RIGHT_B,
    .count_raw = 0,
    .invert = 1
};

static void encoder_update_single(encoder_data_t *encoder)
{
    int16_t count = encoder_get_count(encoder->index);

    encoder_clear_count(encoder->index);

    if (encoder->invert)
    {
        count = -count;
    }

    encoder->count_raw = count;
}

void encoder_control_init(void)
{
    encoder_quad_init(encoder_left.index, encoder_left.ch1_pin, encoder_left.ch2_pin);
    encoder_quad_init(encoder_right.index, encoder_right.ch1_pin, encoder_right.ch2_pin);
}

void encoder_update_100HZ(void)
{
    encoder_update_single(&encoder_left);
    encoder_update_single(&encoder_right);
}

int16_t encoder_get_left_count(void)
{
    return encoder_left.count_raw;
}

int16_t encoder_get_right_count(void)
{
    return encoder_right.count_raw;
}
