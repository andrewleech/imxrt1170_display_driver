#include "py/runtime.h"
#include "py/mphal.h"
#include "lvgl_support.h"

#if !MICROPY_MODULE_BUILTIN_INIT
#error MICROPY_MODULE_BUILTIN_INIT required to be set in build.
#endif

static mp_obj_t ___init___(void) {
    lv_port_disp_init();
    lv_port_indev_init();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(___init___obj, ___init___);

static mp_obj_t deinit(void) {
    // lv_port_indev_deinit();
    lv_port_disp_deinit();
    return mp_const_none;
}
// Define a Python reference to the function above.
static MP_DEFINE_CONST_FUN_OBJ_0(deinit_obj, deinit);


// Define all attributes of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
static const mp_rom_map_elem_t imxrt1170_disp_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_imxrt1170_disp) },
    { MP_ROM_QSTR(MP_QSTR___init__), MP_ROM_PTR(&___init___obj) },
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
