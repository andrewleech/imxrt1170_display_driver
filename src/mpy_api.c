#include "py/runtime.h"
#include "py/mphal.h"
#include "lvgl_support.h"
#include "display_support.h"
#include "fsl_iomuxc.h"
#include "fsl_lpi2c.h"

// Runtime configuration with panel config dict
// Python does ALL I2C initialization before calling this
static mp_obj_t init_with_config(mp_obj_t config_dict) {
    // Validate config_dict is a dict
    if (!mp_obj_is_type(config_dict, &mp_type_dict)) {
        mp_raise_TypeError(MP_ERROR_TEXT("config must be a dict"));
    }

    // Parse panel configuration from Python dict
    panel_config_t config = {0};

    mp_obj_dict_t *dict = MP_OBJ_TO_PTR(config_dict);
    mp_map_t *map = &dict->map;

    // Helper macro to extract dict values (using safe map lookup)
    #define GET_DICT_INT(key, field) do { \
        mp_map_elem_t *elem = mp_map_lookup(map, MP_OBJ_NEW_QSTR(MP_QSTR_##key), MP_MAP_LOOKUP); \
        if (elem != NULL) { \
            config.field = mp_obj_get_int(elem->value); \
        } \
    } while(0)

    #define GET_DICT_STR(key, field) do { \
        mp_map_elem_t *elem = mp_map_lookup(map, MP_OBJ_NEW_QSTR(MP_QSTR_##key), MP_MAP_LOOKUP); \
        if (elem != NULL) { \
            config.field = mp_obj_str_get_str(elem->value); \
        } \
    } while(0)

    // Extract all panel configuration values
    GET_DICT_STR(name, name);
    GET_DICT_INT(width, width);
    GET_DICT_INT(height, height);
    GET_DICT_INT(hsw, hsw);
    GET_DICT_INT(hfp, hfp);
    GET_DICT_INT(hbp, hbp);
    GET_DICT_INT(vsw, vsw);
    GET_DICT_INT(vfp, vfp);
    GET_DICT_INT(vbp, vbp);
    GET_DICT_INT(dsi_lanes, dsi_lanes);

    #undef GET_DICT_INT
    #undef GET_DICT_STR

    // Validate required fields
    if (config.name == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("config must include name"));
    }

    if (config.width == 0 || config.height == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("config must include width and height"));
    }

    if (config.dsi_lanes == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("config must include dsi_lanes"));
    }

    // Validate panel dimensions against buffer size
    if (config.width > DEMO_BUFFER_WIDTH || config.height > DEMO_BUFFER_HEIGHT) {
        mp_raise_ValueError(MP_ERROR_TEXT("panel dimensions exceed buffer size (720x1280)"));
    }

    // Initialize display with runtime configuration
    // Note: Python has already done ALL I2C initialization (Attiny88, PCA6416, etc.)
    // This only sets up MIPI DSI, LCDIF, and LVGL with the panel timing
    lv_port_disp_init_with_config(&config);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(init_with_config_obj, init_with_config);

static mp_obj_t deinit(void) {
    // lv_port_indev_deinit();
    lv_port_disp_deinit();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(deinit_obj, deinit);


// Define all attributes of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
static const mp_rom_map_elem_t imxrt1170_disp_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_imxrt1170_disp) },
    { MP_ROM_QSTR(MP_QSTR_init_with_config), MP_ROM_PTR(&init_with_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&deinit_obj) },
};
static MP_DEFINE_CONST_DICT(imxrt1170_disp_globals, imxrt1170_disp_globals_table);

// Define module object.
const mp_obj_module_t imxrt1170_disp_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&imxrt1170_disp_globals,
};

// Register the module to make it available in Python.
MP_REGISTER_MODULE(MP_QSTR_imxrt1170_disp, imxrt1170_disp_cmodule);
