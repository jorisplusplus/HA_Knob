#include "motor.h"
#include "motorindent.h"
#include "stdlib.h"
#include "esp_log.h"

#define MAX(a, b) (a > b ? a : b)
#define MIN(a, b) (a < b ? a : b)

static motor_indent_t *HEAD = NULL;

/**
 * @brief Register new indent
 * 
 * @param angle Angle in degrees [0, 360] for indent to be placed
 * @param cw Clockwise range the indent will atrack
 * @param ccw Counter clockwise range the indent will attrack
 * @param power Force the indent will apply
 */
void motor_indent_register(float angle, float cw, float ccw, float power) {
    motor_indent_t *new_indent = malloc(sizeof(motor_indent_t));
    new_indent->angle = angle;
    new_indent->cw_range = cw;
    new_indent->ccw_range = ccw;
    new_indent->force = MIN(MAX(0, power), 1);
    new_indent->next = NULL;
    new_indent->prev = NULL;

    if (HEAD == NULL) {
        HEAD = new_indent;
        return;
    }

    motor_indent_t *search = HEAD;
    while (search->next != NULL) {
        search = search->next;
    }

    new_indent->prev = search;
    search->next = new_indent;
}

/**
 * @brief Destroys the list of motor indents, frees all memory used by the list
 * 
 */
void motor_indent_destroy() {
    if (HEAD == NULL) return;
    motor_indent_t *next = HEAD;
    while(next != NULL) {
        motor_indent_t *save;
        save = next;
        next = next->next;
        free(save);
    }
    HEAD = NULL;
}

/**
 * @brief Find the first motor ident where the current angle should snap to.
 * 
 * @param angle ANgle of the motor
 * @return motor_indent_t* First indent in range, NULL when no indent in range
 */
motor_indent_t *motor_indent_find(float angle) {
    static motor_indent_t *prev_indent = NULL;
    
    motor_indent_t *search = HEAD;
    while (search != NULL) {
        float cw_wrap = -1.0f;
        float ccw_wrap = -1.0f;
        float cw_stop = search->angle + search->cw_range;
        float ccw_stop = search->angle - search->ccw_range;

        if (cw_stop > 360.0f) {
            cw_wrap = cw_stop - 360.0f;
            cw_stop = 360.0f;
        }

        if (ccw_stop < 0.0f) {
            ccw_wrap = 360 + ccw_stop;
            ccw_stop = 0.0f;
        }

        if (angle > ccw_stop && angle < cw_stop) {
            if (search != prev_indent) {
                prev_indent = search;
                //ESP_LOGI("motor_indent", "Found indent at %f", search->angle);
            }
            return search;
        }
        //Handle wraparound
        if (cw_wrap > 0.0f && angle < cw_wrap) {
            return search;
        }
        if (ccw_wrap > 0.0f && angle > ccw_wrap) {
            return search;
        }
        search = search->next;
    }
    if (prev_indent != NULL) {
        ESP_LOGI("motor_indent", "No indent at %f", angle);
    }
    prev_indent = NULL;
    return NULL;
}