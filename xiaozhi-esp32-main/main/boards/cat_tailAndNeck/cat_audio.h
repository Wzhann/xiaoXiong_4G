#ifndef _CAT_AUDIO_H_
#define _CAT_AUDIO_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Cat sound types.
 */
typedef enum {
    CAT_SOUND_MEOW = 0,       // 喵~ 经典猫叫
    CAT_SOUND_SHORT_MEOW,     // 短促喵叫
    CAT_SOUND_PURR,           // 呼噜声
    CAT_SOUND_HISS,           // 哈气声
    CAT_SOUND_CHIRP,          // 短促唧唧声（看到鸟）
    CAT_SOUND_TRILL,          // 颤音（友好问候）
    CAT_SOUND_GROWL,          // 低吼
    CAT_SOUND_COUNT
} cat_sound_type_t;

/**
 * @brief Sound description for display/debug.
 */
const char *cat_sound_name(cat_sound_type_t type);

/**
 * @brief Generate a complete cat sound into a newly allocated buffer.
 *
 * Caller must free the returned buffer with free().
 *
 * @param type       Which cat sound to generate.
 * @param sample_rate Sample rate in Hz (e.g. 24000).
 * @param samples_out Number of samples written.
 * @return Heap-allocated int16_t buffer, or NULL on failure.
 */
int16_t *cat_sound_generate(cat_sound_type_t type, int sample_rate, size_t *samples_out);

/**
 * @brief Get the duration of a cat sound in milliseconds.
 */
int cat_sound_duration_ms(cat_sound_type_t type);

#ifdef __cplusplus
}
#endif

#endif // _CAT_AUDIO_H_
