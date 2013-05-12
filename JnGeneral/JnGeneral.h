// This part of the JeeNode General, a single sketch for all your measurement needs!
//
// 2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
// http://opensource.org/licenses/mit-license.php

typedef int16_t measurement_t;

typedef struct {
    void *(*create)(byte port);
    byte (*measure)(void *data, measurement_t *measurements);
    void (*destroy)(void *data);
} PlugInfo;

typedef struct {
    uint8_t unit_id;
    char *descr;
    char *unitname;
} Unit;

typedef struct {
    uint8_t device_id;
    uint8_t num_measurements;
    uint8_t is_i2c;
    struct {
        uint8_t unit;   // The units of the measurements
        uint8_t scale;  // Scale prefix for unit
        uint8_t width;  // bit width. a>0 => 0..2^a-1, a<0 => -2^(a-1)..2^(a-1)-1
    } measurements[4];
    char *name;
    PlugInfo info;
} Device;

typedef struct {
    uint8_t port;
    uint8_t device_id;
    uint8_t period;
    uint8_t id;       // ID of first measurement for this plug, used to match announcements with measurements
    struct {
        uint8_t scale;
        uint8_t width;
    } measurements[4];
    void *data;
} Plug;

