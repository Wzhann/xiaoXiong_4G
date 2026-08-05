#include "cat_audio.h"
#include "cat_audio_samples.h"

#include <stdlib.h>
#include <string.h>

const char *cat_sound_name(cat_sound_type_t type) {
    switch (type) {
        case CAT_SOUND_MEOW:       return "喵~ (Meow)";
        case CAT_SOUND_SHORT_MEOW: return "短喵 (Short Meow)";
        case CAT_SOUND_PURR:       return "呼噜 (Purr)";
        case CAT_SOUND_HISS:       return "哈气 (Hiss)";
        case CAT_SOUND_CHIRP:      return "唧唧 (Chirp)";
        case CAT_SOUND_TRILL:      return "颤音 (Trill)";
        case CAT_SOUND_GROWL:      return "低吼 (Growl)";
        default:                   return "???";
    }
}

int cat_sound_duration_ms(cat_sound_type_t type) {
    if (type >= CAT_SOUND_COUNT) return 500;
    size_t samples = cat_audio_samples[type].length;
    return (int)(samples * 1000 / CAT_AUDIO_SAMPLE_RATE);
}

int16_t *cat_sound_generate(cat_sound_type_t type, int sample_rate, size_t *samples_out) {
    if (type >= CAT_SOUND_COUNT) {
        *samples_out = 0;
        return NULL;
    }

    const cat_audio_sample_t *s = &cat_audio_samples[type];
    if (s->data == NULL || s->length == 0) {
        *samples_out = 0;
        return NULL;
    }

    size_t n = s->length;

    // If the requested sample rate differs, resample (simple linear interpolation)
    if (sample_rate != CAT_AUDIO_SAMPLE_RATE) {
        float ratio = (float)CAT_AUDIO_SAMPLE_RATE / (float)sample_rate;
        n = (size_t)((float)s->length / ratio);
    }

    int16_t *buf = malloc(n * sizeof(int16_t));
    if (buf == NULL) {
        *samples_out = 0;
        return NULL;
    }

    if (sample_rate == CAT_AUDIO_SAMPLE_RATE) {
        // Direct copy — no resampling needed
        memcpy(buf, s->data, n * sizeof(int16_t));
    } else {
        // Linear interpolation resampling
        float ratio = (float)CAT_AUDIO_SAMPLE_RATE / (float)sample_rate;
        for (size_t i = 0; i < n; i++) {
            float src_idx = (float)i * ratio;
            size_t idx0 = (size_t)src_idx;
            size_t idx1 = idx0 + 1;
            if (idx1 >= s->length) idx1 = idx0;
            float frac = src_idx - (float)idx0;
            float val = (float)s->data[idx0] * (1.0f - frac) + (float)s->data[idx1] * frac;
            buf[i] = (int16_t)val;
        }
    }

    *samples_out = n;
    return buf;
}
