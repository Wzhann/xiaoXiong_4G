// Auto-generated cat audio samples — declarations
#ifndef _CAT_AUDIO_SAMPLES_H_
#define _CAT_AUDIO_SAMPLES_H_

#include <stdint.h>
#include <stddef.h>

#define CAT_AUDIO_SAMPLE_RATE 24000

// Extern sample arrays
extern const int16_t cat_sample_meow[];
extern const size_t  cat_sample_meow_len;
extern const int16_t cat_sample_short_meow[];
extern const size_t  cat_sample_short_meow_len;
extern const int16_t cat_sample_purr[];
extern const size_t  cat_sample_purr_len;
extern const int16_t cat_sample_hiss[];
extern const size_t  cat_sample_hiss_len;
extern const int16_t cat_sample_chirp[];
extern const size_t  cat_sample_chirp_len;
extern const int16_t cat_sample_trill[];
extern const size_t  cat_sample_trill_len;
extern const int16_t cat_sample_growl[];
extern const size_t  cat_sample_growl_len;

// Lookup table entry
typedef struct {
    const int16_t *data;
    size_t length;
} cat_audio_sample_t;

// Index by cat_sound_type_t enum
extern const cat_audio_sample_t cat_audio_samples[];

#endif
