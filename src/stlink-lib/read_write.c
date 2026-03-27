/*
 * File: read_write.c
 *
 * Read and write operations
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <stlink.h>
#include <stlink_backend.h>
#include <stm32_register.h>
#include "read_write.h"

#include "helper.h"
#include "logging.h"

static int32_t stlink_wait_reg_ready_memap(stlink_t *sl) {
  uint32_t dhcsr = 0;
  uint32_t timeout = time_ms() + 100;

  while(time_ms() < timeout) {
    if(stlink_read_mem32(sl, STM32_REG_DHCSR, 4) == -1) {
      return (-1);
    }

    dhcsr = read_uint32(sl->q_buf, 0);
    if((dhcsr & STM32_REG_DHCSR_S_REGRDY) != 0) {
      return (0);
    }

    usleep(1000);
  }

  WLOG("Timeout waiting for DHCSR.S_REGRDY on AP%u (DHCSR=0x%08x)\n",
       sl->target_ap, dhcsr);
  return (-1);
}

static int32_t stlink_read_reg_memap(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp) {
  uint32_t r;

  if(r_idx < 0 || r_idx > 18) {
    return sl->backend->read_reg(sl, r_idx, regp);
  }

  sl->q_buf[0] = (unsigned char) r_idx;
  sl->q_buf[1] = 0;
  sl->q_buf[2] = 0;
  sl->q_buf[3] = 0;

  if(stlink_write_mem32(sl, STM32_REG_DCRSR, 4) == -1) {
    return (-1);
  }

  if(stlink_wait_reg_ready_memap(sl) == -1) {
    return (-1);
  }

  if(stlink_read_mem32(sl, STM32_REG_DCRDR, 4) == -1) {
    return (-1);
  }

  r = read_uint32(sl->q_buf, 0);
  DLOG("r_idx (%2d) = 0x%08x\n", r_idx, r);

  switch (r_idx) {
  case 16:
    regp->xpsr = r;
    break;
  case 17:
    regp->main_sp = r;
    break;
  case 18:
    regp->process_sp = r;
    break;
  default:
    regp->r[r_idx] = r;
  }

  return (0);
}

static int32_t stlink_write_reg_memap(stlink_t *sl, uint32_t reg, int32_t idx) {
  if(idx < 0 || idx > 18) {
    return sl->backend->write_reg(sl, reg, idx);
  }

  write_uint32(sl->q_buf, reg);
  if(stlink_write_mem32(sl, STM32_REG_DCRDR, 4) == -1) {
    return (-1);
  }

  sl->q_buf[0] = (unsigned char) idx;
  sl->q_buf[1] = 0;
  sl->q_buf[2] = 0x01;
  sl->q_buf[3] = 0;

  if(stlink_write_mem32(sl, STM32_REG_DCRSR, 4) == -1) {
    return (-1);
  }

  return stlink_wait_reg_ready_memap(sl);
}

static inline bool stlink_h5_native_reg_access_enabled(stlink_t *sl) {
  /* h5_ap1_mode is only set via stlink_h5_enable_ap1_mode(), which is
   * called exclusively for H5 targets.  No additional chip_id / core_id
   * guard is needed here. */
  return sl->h5_ap1_mode;
}

static inline bool stlink_use_routed_ap_reg_access(stlink_t *sl) {
  return stlink_target_uses_ap(sl);
}

static inline bool stlink_h5_is_native_debugreg_addr(uint32_t addr) {
  switch(addr) {
  case STM32_REG_DHCSR:
  case STM32_REG_DCRSR:
  case STM32_REG_DCRDR:
  case STM32_REG_DFSR:
  case STM32_REG_CM3_DEMCR:
  case STM32_REG_AIRCR:
  case STM32_REG_CFSR:
  case STM32_REG_HFSR:
    return true;
  default:
    return false;
  }
}

static inline bool stlink_h5_use_native_debugreg_access(stlink_t *sl, uint32_t addr) {
  return stlink_h5_native_reg_access_enabled(sl) &&
         sl->h5_native_debug_regs &&
         stlink_h5_is_native_debugreg_addr(addr);
}

static bool stlink_h5_can_use_native_core_regs(stlink_t *sl) {
  uint32_t dhcsr = 0;

  if(!(stlink_h5_native_reg_access_enabled(sl) &&
       sl->h5_native_core_regs)) {
    return false;
  }

  if(stlink_read_debug32(sl, STM32_REG_DHCSR, &dhcsr) != 0) {
    return false;
  }

  if((dhcsr & STM32_REG_DHCSR_S_HALT) == 0) {
    DLOG("H5 native core regs not halted (DHCSR=0x%08x), using MEM-AP register access\n",
         dhcsr);
    return false;
  }

  return true;
}

// Endianness
// https://commandcenter.blogspot.com/2012/04/byte-order-fallacy.html
// These functions encode and decode little endian uint16 and uint32 values.

uint16_t read_uint16(const unsigned char *c, const int32_t pt) {
  return ((uint16_t) c[pt]) | ((uint16_t) c[pt + 1] << 8);
}

void write_uint16(unsigned char *buf, uint16_t ui) {
  buf[0] = (uint8_t) ui;
  buf[1] = (uint8_t) (ui >> 8);
}

uint32_t read_uint32(const unsigned char *c, const int32_t pt) {
  return ((uint32_t) c[pt]) | ((uint32_t) c[pt + 1] << 8) |
         ((uint32_t) c[pt + 2] << 16) | ((uint32_t) c[pt + 3] << 24);
}

void write_uint32(unsigned char *buf, uint32_t ui) {
  buf[0] = ui;
  buf[1] = ui >> 8;
  buf[2] = ui >> 16;
  buf[3] = ui >> 24;
}

int32_t stlink_read_debug32(stlink_t *sl, uint32_t addr, uint32_t *data) {
  int32_t ret;

  if(stlink_h5_use_native_debugreg_access(sl, addr)) {
    ret = sl->backend->read_debug32(sl, addr, data);
    if(ret == 0) {
      DLOG("*** stlink_read_debug32  %#010x at %#010x\n", *data, addr);
      return (0);
    }

    WLOG("H5 native READDEBUGREG failed at %#010x, falling back to MEM-AP debug access\n", addr);
    sl->h5_native_debug_regs = false;
  }

  if(stlink_use_routed_ap_reg_access(sl)) {
    ret = stlink_read_mem32(sl, addr, 4);
    if(!ret) {
      *data = read_uint32(sl->q_buf, 0);
    }
  } else {
    ret = sl->backend->read_debug32(sl, addr, data);
  }

  if(!ret) {
    DLOG("*** stlink_read_debug32  %#010x at %#010x\n", *data, addr);
  }

  return (ret);
}

int32_t stlink_write_debug32(stlink_t *sl, uint32_t addr, uint32_t data) {
  DLOG("*** stlink_write_debug32 %#010x to %#010x\n", data, addr);

  if(stlink_h5_use_native_debugreg_access(sl, addr)) {
    int32_t ret = sl->backend->write_debug32(sl, addr, data);
    if(ret == 0) {
      return (0);
    }

    WLOG("H5 native WRITEDEBUGREG failed at %#010x, falling back to MEM-AP debug access\n", addr);
    sl->h5_native_debug_regs = false;
  }

  if(stlink_use_routed_ap_reg_access(sl)) {
    write_uint32(sl->q_buf, data);
    return stlink_write_mem32(sl, addr, 4);
  }

  return sl->backend->write_debug32(sl, addr, data);
}

int32_t stlink_read_mem32(stlink_t *sl, uint32_t addr, uint16_t len) {
  DLOG("*** stlink_read_mem32 ***\n");

  if(len % 4 != 0) { // !!! never ever: fw gives just wrong values
    ELOG("Data length doesn't have a 32 bit alignment: +%d byte.\n", len % 4);
    return (-1);
  }

  return (sl->backend->read_mem32(sl, addr, len));
}

int32_t stlink_write_mem32(stlink_t *sl, uint32_t addr, uint16_t len) {
  DLOG("*** stlink_write_mem32 %u bytes to %#x\n", len, addr);

  if(len % 4 != 0) {
    ELOG("Data length doesn't have a 32 bit alignment: +%d byte.\n", len % 4);
    return (-1);
  }

  return (sl->backend->write_mem32(sl, addr, len));
}

int32_t stlink_write_mem8(stlink_t *sl, uint32_t addr, uint16_t len) {
  DLOG("*** stlink_write_mem8 ***\n");
  return (sl->backend->write_mem8(sl, addr, len));
}

int32_t stlink_read_reg(stlink_t *sl, int32_t r_idx, struct stlink_reg *regp) {
  DLOG("*** stlink_read_reg\n");
  DLOG(" (%d) ***\n", r_idx);

  if(r_idx > 20 || r_idx < 0) {
    fprintf(stderr, "Error: register index must be in [0..20]\n");
    return (-1);
  }

  if(stlink_h5_can_use_native_core_regs(sl)) {
    int32_t ret = sl->backend->read_reg(sl, r_idx, regp);
    if(ret == 0) {
      return (0);
    }

    WLOG("H5 native READREG failed for r%d, falling back to MEM-AP register access\n", r_idx);
    sl->h5_native_core_regs = false;
  }

  if(stlink_use_routed_ap_reg_access(sl)) {
    return stlink_read_reg_memap(sl, r_idx, regp);
  }

  return (sl->backend->read_reg(sl, r_idx, regp));
}

int32_t stlink_write_reg(stlink_t *sl, uint32_t reg, int32_t idx) {
  DLOG("*** stlink_write_reg\n");

  if(stlink_h5_can_use_native_core_regs(sl)) {
    int32_t ret = sl->backend->write_reg(sl, reg, idx);
    if(ret == 0) {
      return (0);
    }

    WLOG("H5 native WRITEREG failed for r%d, falling back to MEM-AP register access\n", idx);
    sl->h5_native_core_regs = false;
  }

  if(stlink_use_routed_ap_reg_access(sl)) {
    return stlink_write_reg_memap(sl, reg, idx);
  }

  return (sl->backend->write_reg(sl, reg, idx));
}

int32_t stlink_read_unsupported_reg(stlink_t *sl, int32_t r_idx,
                                struct stlink_reg *regp) {
  int32_t r_convert;

  DLOG("*** stlink_read_unsupported_reg\n");
  DLOG(" (%d) ***\n", r_idx);

  /* Convert to values used by STM32_REG_DCRSR */
  if(r_idx >= 0x1C &&
      r_idx <= 0x1F) { // primask, basepri, faultmask, or control
    r_convert = 0x14;
  } else if(r_idx == 0x40) { // FPSCR
    r_convert = 0x21;
  } else if(r_idx >= 0x20 && r_idx < 0x40) {
    r_convert = 0x40 + (r_idx - 0x20);
  } else {
    fprintf(stderr, "Error: register address must be in [0x1C..0x40]\n");
    return (-1);
  }

  return (sl->backend->read_unsupported_reg(sl, r_convert, regp));
}

int32_t stlink_write_unsupported_reg(stlink_t *sl, uint32_t val, int32_t r_idx,
                                 struct stlink_reg *regp) {
  int32_t r_convert;

  DLOG("*** stlink_write_unsupported_reg\n");
  DLOG(" (%d) ***\n", r_idx);

  /* Convert to values used by STM32_REG_DCRSR */
  if(r_idx >= 0x1C &&
      r_idx <= 0x1F) {        /* primask, basepri, faultmask, or control */
    r_convert = r_idx;        // the backend function handles this
  } else if(r_idx == 0x40) { // FPSCR
    r_convert = 0x21;
  } else if(r_idx >= 0x20 && r_idx < 0x40) {
    r_convert = 0x40 + (r_idx - 0x20);
  } else {
    fprintf(stderr, "Error: register address must be in [0x1C..0x40]\n");
    return (-1);
  }

  return (sl->backend->write_unsupported_reg(sl, val, r_convert, regp));
}

int32_t stlink_read_all_regs(stlink_t *sl, struct stlink_reg *regp) {
  DLOG("*** stlink_read_all_regs ***\n");

  if(stlink_use_routed_ap_reg_access(sl)) {
    memset(regp, 0, sizeof(*regp));

    for(int32_t i = 0; i <= 18; i++) {
      if(stlink_read_reg(sl, i, regp) == -1) {
        return (-1);
      }
    }

    return (0);
  }

  return (sl->backend->read_all_regs(sl, regp));
}

int32_t stlink_read_all_unsupported_regs(stlink_t *sl, struct stlink_reg *regp) {
  DLOG("*** stlink_read_all_unsupported_regs ***\n");
  return (sl->backend->read_all_unsupported_regs(sl, regp));
}
