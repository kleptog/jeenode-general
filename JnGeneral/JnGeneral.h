// This part of the JeeNode General, a single sketch for all your measurement needs!
//
// 2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
// http://opensource.org/licenses/mit-license.php

typedef struct {
    void *(*create)(int port);
    int (*measure)(void *data, int32_t *measurements);
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
        uint8_t unit;
        uint8_t scale;
    } measurements[4];  // The units of the measurements
    char *name;
    PlugInfo info;
} Device;

typedef struct {
    uint8_t port;
    uint8_t device_id;
    uint8_t period;
    uint8_t scale[4];
    void *data;
} Plug;

