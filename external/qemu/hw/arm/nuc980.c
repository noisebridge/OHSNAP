/*
 * ARM Versatile Platform/Application Baseboard System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qom/object.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "hw/clock.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/arm/boot.h"
#include "hw/char/serial.h"
#include "hw/block/flash.h"
#include "hw/ssi/ssi.h"

/*****************************************************************************/
/*                               BOOT FLASH                                  */
/*****************************************************************************/

static uint8_t nuc980_boot_flash[32*1024*1024];

/*****************************************************************************/
/*                             SYSTEM CONTROLLER                             */
/*****************************************************************************/

/*---------------------------- SYS MODULE NAME ------------------------------*/

#define TYPE_NUC980_SYS "nuc980-sys"

/*----------------------------- SYS REGISTERS -------------------------------*/

#define REG_SYS_PDID       0x000
#define REG_SYS_PWRON      0x004
#define REG_SYS_ARBCON     0x008
#define REG_SYS_LVRDCR     0x020
#define REG_SYS_MISCFCR    0x030
#define REG_SYS_MISCIER    0x040
#define REG_SYS_MISCISR    0x044
#define REG_SYS_ROMSUM0    0x048
#define REG_SYS_ROMSUM1    0x04C
#define REG_SYS_WKUPSER    0x058
#define REG_SYS_WKUPSSR    0x05C
#define REG_SYS_AHBIPRST   0x060
#define REG_SYS_APBIPRST0  0x064
#define REG_SYS_APBIPRST1  0x068
#define REG_SYS_RSTSTS     0x06C
#define REG_SYS_MFP_GPA_L  0x070
#define REG_SYS_MFP_GPA_H  0x074
#define REG_SYS_MFP_GPB_L  0x078
#define REG_SYS_MFP_GPB_H  0x07C
#define REG_SYS_MFP_GPC_L  0x080
#define REG_SYS_MFP_GPC_H  0x084
#define REG_SYS_MFP_GPD_L  0x088
#define REG_SYS_MFP_GPD_H  0x08C
#define REG_SYS_MFP_GPE_L  0x090
#define REG_SYS_MFP_GPE_H  0x094
#define REG_SYS_MFP_GPF_L  0x098
#define REG_SYS_MFP_GPF_H  0x09C
#define REG_SYS_MFP_GPG_L  0x0A0
#define REG_SYS_MFP_GPG_H  0x0A4
#define REG_SYS_MFP_GPH_L  0x0A8
#define REG_SYS_MFP_GPH_H  0x0AC
#define REG_SYS_MFP_GPI_L  0x0B0
#define REG_SYS_MFP_GPI_H  0x0B4
#define REG_SYS_DDR_DSCTL  0x0F0
#define REG_SYS_GPBL_DSCTL 0x0F4
#define REG_SYS_PORDISCR   0x100
#define REG_SYS_RSTDEBCTL  0x10C
#define REG_SYS_WPCTL      0x1FC

/*---------------------------- SYS DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SYSState, NUC980_SYS)

struct NUC980SYSState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    uint32_t      regs[256];
};

/*----------------------------- SYS FUNCTIONS -------------------------------*/

static uint64_t nuc980_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SYSState *sys = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_SYS_PDID:
        break;

      case REG_SYS_PWRON:
        /* boot source:          0b10  (NAND flash) 
         * QSPI clock:           0b0   (37.5MHz)
         * Watchdog:             0b0   (disabled)
         * JTAG:                 0b1   (PinG)
         * UART debug:           0b1   (OFF)
         * NAND flash page size: 0b11  (ignore)
         * MISC config:          0b11  (ignore)
         * USB ID:               0b0   (USB device)
         * TIC Mode:             0b0   (Disabled)
         * DRAM Size:            0b110 (64MB)
         */
        ret = 0x004003F2;
        break;

      case REG_SYS_ARBCON:
      case REG_SYS_LVRDCR:
      case REG_SYS_MISCFCR:
      case REG_SYS_MISCIER:
      case REG_SYS_MISCISR:
      case REG_SYS_ROMSUM0:
      case REG_SYS_ROMSUM1:
      case REG_SYS_WKUPSER:
      case REG_SYS_WKUPSSR:
      case REG_SYS_AHBIPRST:
      case REG_SYS_APBIPRST0:
      case REG_SYS_APBIPRST1:
      case REG_SYS_RSTSTS:
      case REG_SYS_MFP_GPA_L:
      case REG_SYS_MFP_GPA_H:
      case REG_SYS_MFP_GPB_L:
      case REG_SYS_MFP_GPB_H:
      case REG_SYS_MFP_GPC_L:
      case REG_SYS_MFP_GPC_H:
      case REG_SYS_MFP_GPD_L:
      case REG_SYS_MFP_GPD_H:
      case REG_SYS_MFP_GPE_L:
      case REG_SYS_MFP_GPE_H:
      case REG_SYS_MFP_GPF_L:
      case REG_SYS_MFP_GPF_H:
      case REG_SYS_MFP_GPG_L:
      case REG_SYS_MFP_GPG_H:
      case REG_SYS_MFP_GPH_L:
      case REG_SYS_MFP_GPH_H:
      case REG_SYS_MFP_GPI_L:
      case REG_SYS_MFP_GPI_H:
      case REG_SYS_DDR_DSCTL:
      case REG_SYS_GPBL_DSCTL:
      case REG_SYS_PORDISCR:
      case REG_SYS_RSTDEBCTL:
        ret = sys->regs[(addr>>2)&0xFF];
        break;

      case REG_SYS_WPCTL:
        ret = 1;
        break;

      default:
        error_report("SYS RD: 0x%08lX --> 0x%08lX", sys->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_sys_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SYSState *sys = opaque;

    switch(addr) {
      case REG_SYS_PDID:
        break;

      case REG_SYS_PWRON:
        break;

      case REG_SYS_ARBCON:
      case REG_SYS_LVRDCR:
      case REG_SYS_MISCFCR:
      case REG_SYS_MISCIER:
      case REG_SYS_MISCISR:
      case REG_SYS_ROMSUM0:
      case REG_SYS_ROMSUM1:
      case REG_SYS_WKUPSER:
      case REG_SYS_WKUPSSR:
      case REG_SYS_AHBIPRST:
      case REG_SYS_APBIPRST0:
      case REG_SYS_APBIPRST1:
      case REG_SYS_RSTSTS:
      case REG_SYS_MFP_GPA_L:
      case REG_SYS_MFP_GPA_H:
      case REG_SYS_MFP_GPB_L:
      case REG_SYS_MFP_GPB_H:
      case REG_SYS_MFP_GPC_L:
      case REG_SYS_MFP_GPC_H:
      case REG_SYS_MFP_GPD_L:
      case REG_SYS_MFP_GPD_H:
      case REG_SYS_MFP_GPE_L:
      case REG_SYS_MFP_GPE_H:
      case REG_SYS_MFP_GPF_L:
      case REG_SYS_MFP_GPF_H:
      case REG_SYS_MFP_GPG_L:
      case REG_SYS_MFP_GPG_H:
      case REG_SYS_MFP_GPH_L:
      case REG_SYS_MFP_GPH_H:
      case REG_SYS_MFP_GPI_L:
      case REG_SYS_MFP_GPI_H:
      case REG_SYS_DDR_DSCTL:
      case REG_SYS_GPBL_DSCTL:
      case REG_SYS_PORDISCR:
      case REG_SYS_RSTDEBCTL:
        sys->regs[(addr>>2)&0xFF] = value;
        break;

      case REG_SYS_WPCTL:
        break;

      default:
        error_report("SYS WR: 0x%08lX <-- 0x%08lX", sys->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_sys_instance_init(Object *obj)
{
    NUC980SYSState *sys = NUC980_SYS(obj);
    
    static const MemoryRegionOps sys_ops = {
      .read       = nuc980_sys_read,
      .write      = nuc980_sys_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&sys->iomem, obj, &sys_ops, sys, "sys", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(sys), &sys->iomem);
}

static void nuc980_sys_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SYS Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SYS TYPE -----------------------------------*/

static const TypeInfo nuc980_sys_type = {
    .name          = TYPE_NUC980_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SYSState),
    .instance_init = nuc980_sys_instance_init,
    .class_init    = nuc980_sys_class_init,
};

/*****************************************************************************/
/*                             CLOCK CONTROLLER                              */
/*****************************************************************************/

/*--------------------------- CLK MODULE NAME -------------------------------*/

#define TYPE_NUC980_CLK "nuc980-clk"

#define REG_CLK_PMCON       0x000
#define REG_CLK_HCLKEN      0x010
#define REG_CLK_PCLKEN0     0x018
#define REG_CLK_PCLKEN1     0x01C
#define REG_CLK_DIVCTL0     0x020
#define REG_CLK_DIVCTL1     0x024
#define REG_CLK_DIVCTL2     0x028
#define REG_CLK_DIVCTL3     0x02C
#define REG_CLK_DIVCTL4     0x030
#define REG_CLK_DIVCTL5     0x034
#define REG_CLK_DIVCTL6     0x038
#define REG_CLK_DIVCTL7     0x03C
#define REG_CLK_DIVCTL8     0x040
#define REG_CLK_DIVCTL9     0x044
#define REG_CLK_APLLCON     0x060
#define REG_CLK_UPLLCON     0x064
#define REG_CLK_PLLSTBC     0x080

/*---------------------------- CLK DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980CLKState, NUC980_CLK)

struct NUC980CLKState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    uint32_t      reg_pmctl;
    uint32_t      reg_hclken;
    uint32_t      reg_pclken[2];
    uint32_t      reg_divctl[10];
    uint32_t      reg_pllctl[2];
    uint32_t      reg_pllstbc;
    Clock        *clk_xtal[2];
    Clock        *clk_pll[2];
    Clock        *clk_sys;
    Clock        *clk_cpu;
    Clock        *clk_hclk[5];
    Clock        *clk_pclk[3];
    Clock        *clk_timer[6];
};

/*----------------------------- CLK FUNCTIONS -------------------------------*/

static void nuc980_clk_update(void *opaque)
{
    NUC980CLKState *clk = opaque;
    uint32_t        div = 0;
    uint32_t        i = 0;

    /* start with external crystal oscillators */
    clock_set_hz(clk->clk_xtal[0], 12000000);
    clock_set_hz(clk->clk_xtal[1], 32768);
    
    /* compute sysclk */
    div = ((clk->reg_divctl[0]>>8) & 1) + 1;
    switch (((clk->reg_divctl[0]>>3) & 3)) {
      case 0:
        clock_set_hz(clk->clk_sys, clock_get_hz(clk->clk_xtal[0])/div);
        break;

      case 1:
        clock_set_hz(clk->clk_sys, 0);
        break;

      case 2:
        clock_set_hz(clk->clk_sys, clock_get_hz(clk->clk_pll[0])/div);
        break;

      case 3:
        clock_set_hz(clk->clk_sys, clock_get_hz(clk->clk_pll[1])/div);
        break;
        
      default:
        break;
    }
    
    /* compute cpuclk */
    div = ((clk->reg_divctl[0]>>16) & 1) + 1;
    clock_set_hz(clk->clk_cpu, clock_get_hz(clk->clk_sys)/div);
    
    /* compute hclk# */
    clock_set_hz(clk->clk_hclk[0], clock_get_hz(clk->clk_sys)/2);
    clock_set_hz(clk->clk_hclk[1], clock_get_hz(clk->clk_sys)/2);
    clock_set_hz(clk->clk_hclk[2], clock_get_hz(clk->clk_sys)/2);
    clock_set_hz(clk->clk_hclk[3], clock_get_hz(clk->clk_sys)/2);
    clock_set_hz(clk->clk_hclk[4], clock_get_hz(clk->clk_sys)/2);
    
    /* compute pclk# */
    clock_set_hz(clk->clk_pclk[0], clock_get_hz(clk->clk_hclk[1])/1);
    clock_set_hz(clk->clk_pclk[1], clock_get_hz(clk->clk_hclk[1])/1);
    clock_set_hz(clk->clk_pclk[2], clock_get_hz(clk->clk_hclk[1])/2);
    
    /* compute timer# */
    for (i = 0; i < 6; i++) {
      switch (((clk->reg_divctl[8]>>(16+i*2)) & 3)) {
        case 0:
          clock_set_hz(clk->clk_timer[i], clock_get_hz(clk->clk_xtal[0])/1);
          break;

        case 1:
          clock_set_hz(clk->clk_timer[i], clock_get_hz(clk->clk_pclk[0])/1);
          break;

        case 2:
          clock_set_hz(clk->clk_timer[i], clock_get_hz(clk->clk_pclk[0])/4096);
          break;

        case 3:
          clock_set_hz(clk->clk_timer[i], clock_get_hz(clk->clk_xtal[1])/1);
          break;
        
        default:
          break;
      }
    }
}

static uint64_t nuc980_clk_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980CLKState *clk = opaque;
    uint64_t        ret = 0;

    switch (addr) {
      case REG_CLK_PMCON:
        ret = clk->reg_pmctl;
        break;

      case REG_CLK_HCLKEN:
        ret = clk->reg_hclken;
        break;

      case REG_CLK_PCLKEN0:
        ret = clk->reg_pclken[0];
        break;

      case REG_CLK_PCLKEN1:
        ret = clk->reg_pclken[1];
        break;

      case REG_CLK_DIVCTL0:
        ret = clk->reg_divctl[0];
        break;

      case REG_CLK_DIVCTL1:
        ret = clk->reg_divctl[1];
        break;

      case REG_CLK_DIVCTL2:
        ret = clk->reg_divctl[2];
        break;

      case REG_CLK_DIVCTL3:
        ret = clk->reg_divctl[3];
        break;

      case REG_CLK_DIVCTL4:
        ret = clk->reg_divctl[4];
        break;

      case REG_CLK_DIVCTL5:
        ret = clk->reg_divctl[5];
        break;

      case REG_CLK_DIVCTL6:
        ret = clk->reg_divctl[6];
        break;

      case REG_CLK_DIVCTL7:
        ret = clk->reg_divctl[7];
        break;

      case REG_CLK_DIVCTL8:
        ret = clk->reg_divctl[8];
        break;

      case REG_CLK_DIVCTL9:
        ret = clk->reg_divctl[9];
        break;

      case REG_CLK_APLLCON:
        ret = clk->reg_pllctl[0];
        break;

      case REG_CLK_UPLLCON:
        ret = clk->reg_pllctl[1];
        break;

      case REG_CLK_PLLSTBC:
        ret = clk->reg_pllstbc;
        break;
    
      default:
        error_report("CLK RD: 0x%08lX --> 0x%08lX", clk->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_clk_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980CLKState *clk = opaque;

    switch (addr) {
      case REG_CLK_PMCON:
        clk->reg_pmctl = value;
        break;

      case REG_CLK_HCLKEN:
        clk->reg_hclken = value;
        break;

      case REG_CLK_PCLKEN0:
        clk->reg_pclken[0] = value;
        break;

      case REG_CLK_PCLKEN1:
        clk->reg_pclken[1] = value;
        break;

      case REG_CLK_DIVCTL0:
        clk->reg_divctl[0] = value;
        break;

      case REG_CLK_DIVCTL1:
        clk->reg_divctl[1] = value;
        break;

      case REG_CLK_DIVCTL2:
        clk->reg_divctl[2] = value;
        break;

      case REG_CLK_DIVCTL3:
        clk->reg_divctl[3] = value;
        break;

      case REG_CLK_DIVCTL4:
        clk->reg_divctl[4] = value;
        break;

      case REG_CLK_DIVCTL5:
        clk->reg_divctl[5] = value;
        break;

      case REG_CLK_DIVCTL6:
        clk->reg_divctl[6] = value;
        break;

      case REG_CLK_DIVCTL7:
        clk->reg_divctl[7] = value;
        break;

      case REG_CLK_DIVCTL8:
        clk->reg_divctl[8] = value;
        break;

      case REG_CLK_DIVCTL9:
        clk->reg_divctl[9] = value;
        break;

      case REG_CLK_APLLCON:
        clk->reg_pllctl[0] = value;
        break;

      case REG_CLK_UPLLCON:
        clk->reg_pllctl[1] = value;
        break;

      case REG_CLK_PLLSTBC:
        clk->reg_pllstbc = value;
        break;
    
      default:
        error_report("CLK WR: 0x%08lX <-- 0x%08lX", clk->iomem.addr + addr, value);
        break;
    }
    
    /* update clocks recursively */
    nuc980_clk_update(clk);
}

static void nuc980_clk_instance_init(Object *obj)
{
    NUC980CLKState *clk = NUC980_CLK(obj);
    
    static const MemoryRegionOps clk_ops = {
      .read       = nuc980_clk_read,
      .write      = nuc980_clk_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&clk->iomem, obj, &clk_ops, clk, "clk", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(clk), &clk->iomem);

    clk->clk_xtal[0] = clock_new(OBJECT(clk), "xtal_12m");
    clk->clk_xtal[1] = clock_new(OBJECT(clk), "xtal_32k");
    clk->clk_pll[0]   = clock_new(OBJECT(clk), "pll[0]");
    clk->clk_pll[1]   = clock_new(OBJECT(clk), "pll[1]");
    clk->clk_sys      = clock_new(OBJECT(clk), "sys");
    clk->clk_cpu      = clock_new(OBJECT(clk), "cpu");
    clk->clk_hclk[0]  = clock_new(OBJECT(clk), "hclk[0]");
    clk->clk_hclk[1]  = clock_new(OBJECT(clk), "hclk[1]");
    clk->clk_hclk[2]  = clock_new(OBJECT(clk), "hclk[2]");
    clk->clk_hclk[3]  = clock_new(OBJECT(clk), "hclk[3]");
    clk->clk_hclk[4]  = clock_new(OBJECT(clk), "hclk[4]");
    clk->clk_pclk[0]  = clock_new(OBJECT(clk), "pclk[0]");
    clk->clk_pclk[1]  = clock_new(OBJECT(clk), "pclk[1]");
    clk->clk_pclk[2]  = clock_new(OBJECT(clk), "pclk[2]");
    clk->clk_timer[0] = clock_new(OBJECT(clk), "timer[0]");
    clk->clk_timer[1] = clock_new(OBJECT(clk), "timer[1]");
    clk->clk_timer[2] = clock_new(OBJECT(clk), "timer[2]");
    clk->clk_timer[3] = clock_new(OBJECT(clk), "timer[3]");
    clk->clk_timer[4] = clock_new(OBJECT(clk), "timer[4]");
    clk->clk_timer[5] = clock_new(OBJECT(clk), "timer[5]");
}

static void nuc980_clk_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 CLK Controller";
    dev_class->reset = NULL;
}

/*------------------------------ CLK TYPE -----------------------------------*/

static const TypeInfo nuc980_clk_type = {
    .name          = TYPE_NUC980_CLK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980CLKState),
    .instance_init = nuc980_clk_instance_init,
    .class_init    = nuc980_clk_class_init,
};

/*****************************************************************************/
/*                          SDRAM INTERFACE CONTROLLER                       */
/*****************************************************************************/

/*----------------------------- SDR MODULE NAME -----------------------------*/

#define TYPE_NUC980_SDR "nuc980-sdr"

/*----------------------------- SDR REGISTERS -------------------------------*/

#define REG_SDR_OPMCTL     0x000
#define REG_SDR_CMD        0x004
#define REG_SDR_REFCTL     0x008
#define REG_SDR_SIZE0      0x010
#define REG_SDR_SIZE1      0x014
#define REG_SDR_MR         0x018
#define REG_SDR_EMR        0x01C
#define REG_SDR_EMR2       0x020
#define REG_SDR_EMR3       0x024
#define REG_SDR_TIME       0x028
#define REG_SDR_DQSODS     0x030
#define REG_SDR_CKDQSDS    0x034
#define REG_SDR_DAENSEL    0x038

/*---------------------------- SDR DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SDRState, NUC980_SDR)

struct NUC980SDRState {
    SysBusDevice  parent_obj;
    MemoryRegion  sdram;
    MemoryRegion  bootram;
    MemoryRegion  iomem;
    SSIBus       *bus;
    uint32_t      lol_reg;
};

/*----------------------------- SDR FUNCTIONS -------------------------------*/

static uint64_t nuc980_sdr_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SDRState *sdr = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_SDR_SIZE0:
        ret = 0x0000000E; // SDRAM0: 64MB
        break;

      case REG_SDR_SIZE1:
        ret = 0x00000000; // SDRAM1: 0MB
        break;

      default:
        error_report("SDR RD: 0x%08lX --> 0x%08lX", sdr->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_sdr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SDRState *sdr = opaque;

    switch(addr) {
      default:
        error_report("SDR WR: 0x%08lX <-- 0x%08lX", sdr->iomem.addr + addr, value);
        break;
    }    
}

static void nuc980_sdr_instance_init(Object *obj)
{
    NUC980SDRState *sdr     = NUC980_SDR(obj);

    static const MemoryRegionOps sdr_ops = {
      .read       = nuc980_sdr_read,
      .write      = nuc980_sdr_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_ram(&sdr->sdram, NULL, "sdram", 64*1024*1024, &error_fatal);
    memory_region_init_ram(&sdr->bootram, NULL, "bootram", 0x4000, &error_fatal);
    memory_region_init_io(&sdr->iomem, obj, &sdr_ops, sdr, "sdr", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(sdr), &sdr->iomem);
}

static void nuc980_sdr_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SDR Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SDR TYPE -----------------------------------*/

static const TypeInfo nuc980_sdr_type = {
    .name          = TYPE_NUC980_SDR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SDRState),
    .instance_init = nuc980_sdr_instance_init,
    .class_init    = nuc980_sdr_class_init,
};

/*****************************************************************************/
/*                              IRQ CONTROLLER                               */
/*****************************************************************************/

/*---------------------------- PIC MODULE NAME ------------------------------*/

#define TYPE_NUC980_PIC "nuc980-pic"

/*----------------------------- PIC REGISTERS -------------------------------*/

#define REG_PIC_SRC00    0x000
#define REG_PIC_SRC01    0x004
#define REG_PIC_SRC02    0x008
#define REG_PIC_SRC03    0x00C
#define REG_PIC_SRC04    0x010
#define REG_PIC_SRC05    0x014
#define REG_PIC_SRC06    0x018
#define REG_PIC_SRC07    0x01C
#define REG_PIC_SRC08    0x020
#define REG_PIC_SRC09    0x024
#define REG_PIC_SRC10    0x028
#define REG_PIC_SRC11    0x02C
#define REG_PIC_SRC12    0x030
#define REG_PIC_SRC13    0x034
#define REG_PIC_SRC14    0x038
#define REG_PIC_SRC15    0x03C
#define REG_PIC_RAW0     0x100
#define REG_PIC_RAW1     0x104
#define REG_PIC_IS0      0x110
#define REG_PIC_IS1      0x114
#define REG_PIC_IRQ      0x120
#define REG_PIC_FIQ      0x124
#define REG_PIC_IE0      0x128
#define REG_PIC_IE1      0x12C
#define REG_PIC_IEN0     0x130
#define REG_PIC_IEN1     0x134
#define REG_PIC_IDIS0    0x138
#define REG_PIC_IDIS1    0x13C
#define REG_PIC_IRQRST   0x150
#define REG_PIC_FIQRST   0x154

/*---------------------------- PIC DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980PICState, NUC980_PIC)

struct NUC980PICState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq[2];
    uint64_t      int_ien;
    uint64_t      int_sts;
    uint8_t       irq_src;
    uint8_t       fiq_src;
    uint8_t       irq_pin;
    uint8_t       fiq_pin;
};

/*----------------------------- PIC FUNCTIONS -------------------------------*/

static void nuc980_pic_update(void *opaque)
{
    NUC980PICState *pic = opaque;  
    int             i   = 0; 

    /* should there be any IRQ to CPU? */
    if (pic->int_sts) {
      for (i = 0; i < 64; i++) {
        if (pic->int_sts & (1UL<<i)) {
          pic->irq_src = i;
        }
      }
      if (pic->irq_pin == 0) {
        qemu_set_irq(pic->irq[0], 1);
        pic->irq_pin = 1;
        //printf("IRQ SOURCE: %d\n", pic->irq_src);
      }
    } else {
      pic->irq_src = 0;
      if (pic->irq_pin == 1) {
        qemu_set_irq(pic->irq[0], 0);
        pic->irq_pin = 0;
        //printf("IRQ CLEARED\n");
      }    
    }
}

static void nuc980_pic_irq(void *opaque, int irq, int level)
{
    NUC980PICState *pic = opaque;

    if (level == 0) {
      pic->int_sts &= ~(1UL<<irq);
    } else {
      // FIXME: when timer counter is too small, something missy happens
      pic->int_sts |= (1UL<<irq); //& pic->int_ien;
    }
    //printf("IRQ: %d %d 0x%016lX 0x%016lX\n", irq, level, pic->int_sts, pic->int_ien);

    nuc980_pic_update(pic);
}

static uint64_t nuc980_pic_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980PICState *pic = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_PIC_RAW0:
        ret = (pic->int_sts>>0) & 0xFFFFFFFF;
        break;

      case REG_PIC_RAW1:
        ret = (pic->int_sts>>32) & 0xFFFFFFFF;
        break;

      case REG_PIC_IS0:
        ret = (pic->int_sts>>0) & 0xFFFFFFFF;
        break;

      case REG_PIC_IS1:
        ret = (pic->int_sts>>32) & 0xFFFFFFFF;
        break;

      case REG_PIC_IRQ:
        ret = pic->irq_src;
        break;

      case REG_PIC_FIQ:
        ret = pic->fiq_src;
        break;

      case REG_PIC_IE0:
        ret = (pic->int_ien>>0) & 0xFFFFFFFF;
        break;

      case REG_PIC_IE1:
        ret = (pic->int_ien>>32) & 0xFFFFFFFF;
        break;

      default:
        error_report("PIC RD: 0x%08lX --> 0x%08lX", pic->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_pic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980PICState *pic = opaque;

    switch(addr) {
      case REG_PIC_IE0:
        pic->int_ien = (pic->int_ien & 0xFFFFFFFF00000000ULL) | (value<< 0);
        break;

      case REG_PIC_IE1:
        pic->int_ien = (pic->int_ien & 0x00000000FFFFFFFFULL) | (value<<32);
        break;

      case REG_PIC_IEN0:
        pic->int_ien |= (value<< 0);
        break;

      case REG_PIC_IEN1:
        pic->int_ien |= (value<<32);
        break;

      case REG_PIC_IDIS0:
        pic->int_ien &= ~(value<< 0);
        break;

      case REG_PIC_IDIS1:
        pic->int_ien &= ~(value<<32);
        break;

      case REG_PIC_IRQRST:  
        nuc980_pic_update(pic);
        break;

      case REG_PIC_FIQRST:
        break;

      default:
        error_report("PIC WR: 0x%08lX <-- 0x%08lX", pic->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_pic_instance_init(Object *obj)
{
    NUC980PICState *pic = NUC980_PIC(obj);
    
    static const MemoryRegionOps pic_ops = {
      .read       = nuc980_pic_read,
      .write      = nuc980_pic_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&pic->iomem, obj, &pic_ops, pic, "pic", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(pic), &pic->iomem);

    qdev_init_gpio_in (DEVICE(pic), nuc980_pic_irq, 64);
    qdev_init_gpio_out(DEVICE(pic), pic->irq,       2);
}

static void nuc980_pic_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 PIC Controller";
    dev_class->reset = NULL;
}

/*------------------------------ PIC TYPE -----------------------------------*/

static const TypeInfo nuc980_pic_type = {
    .name          = TYPE_NUC980_PIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980PICState),
    .instance_init = nuc980_pic_instance_init,
    .class_init    = nuc980_pic_class_init,
};

/*****************************************************************************/
/*                               GPIO DRIVER                                 */
/*****************************************************************************/

/*---------------------------- GPI MODULE NAME ------------------------------*/

#define TYPE_NUC980_GPI "nuc980-gpi"

/*----------------------------- GPI REGISTERS -------------------------------*/

#define REG_GPI_MODE       0x000
#define REG_GPI_DINOFF     0x004
#define REG_GPI_DOUT       0x008
#define REG_GPI_DATMSK     0x00C
#define REG_GPI_PIN        0x010
#define REG_GPI_DBEN       0x014
#define REG_GPI_INTTYPE    0x018
#define REG_GPI_INTEN      0x01C
#define REG_GPI_INTSRC     0x020
#define REG_GPI_SMTEN      0x024
#define REG_GPI_SLEWCTL    0x028
#define REG_GPI_PUSEL      0x030

#define REG_GPI_DATA0      0x000
#define REG_GPI_DATA1      0x004
#define REG_GPI_DATA2      0x008
#define REG_GPI_DATA3      0x00C
#define REG_GPI_DATA4      0x010
#define REG_GPI_DATA5      0x014
#define REG_GPI_DATA6      0x018
#define REG_GPI_DATA7      0x01C
#define REG_GPI_DATA8      0x020
#define REG_GPI_DATA9      0x024
#define REG_GPI_DATA10     0x028
#define REG_GPI_DATA11     0x02C
#define REG_GPI_DATA12     0x030
#define REG_GPI_DATA13     0x034
#define REG_GPI_DATA14     0x038
#define REG_GPI_DATA15     0x03C

/*---------------------------- GPI DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980GPIState, NUC980_GPI)

struct NUC980GPIState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem[2];
    qemu_irq      irq;
};

/*----------------------------- GPI FUNCTIONS -------------------------------*/

static uint64_t nuc980_gpi_read_reg(void *opaque, hwaddr addr, unsigned size)
{
    NUC980GPIState *gpi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_GPI_DINOFF:
      case REG_GPI_DOUT:
      case REG_GPI_DATMSK:
      case REG_GPI_PIN:
      case REG_GPI_DBEN:
      case REG_GPI_INTTYPE:
      case REG_GPI_INTEN:
      case REG_GPI_INTSRC:
      case REG_GPI_SMTEN:
      case REG_GPI_SLEWCTL:
      case REG_GPI_PUSEL:
        break;

      default:
        error_report("GPI RD: 0x%08lX --> 0x%08lX", gpi->iomem[0].addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_gpi_write_reg(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980GPIState *gpi = opaque;

    switch(addr) {
      case REG_GPI_DINOFF:
      case REG_GPI_DOUT:
      case REG_GPI_DATMSK:
      case REG_GPI_PIN:
      case REG_GPI_DBEN:
      case REG_GPI_INTTYPE:
      case REG_GPI_INTEN:
      case REG_GPI_INTSRC:
      case REG_GPI_SMTEN:
      case REG_GPI_SLEWCTL:
      case REG_GPI_PUSEL:
        break;

      default:
        error_report("GPI WR: 0x%08lX <-- 0x%08lX", gpi->iomem[0].addr + addr, value);
        break;
    }
}

static uint64_t nuc980_gpi_read_data(void *opaque, hwaddr addr, unsigned size)
{
    NUC980GPIState *gpi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_GPI_DATA0:
      case REG_GPI_DATA1:
      case REG_GPI_DATA2:
      case REG_GPI_DATA3:
      case REG_GPI_DATA4:
      case REG_GPI_DATA5:
      case REG_GPI_DATA6:
      case REG_GPI_DATA7:
      case REG_GPI_DATA8:
      case REG_GPI_DATA9:
      case REG_GPI_DATA10:
      case REG_GPI_DATA11:
      case REG_GPI_DATA12:
      case REG_GPI_DATA13:
      case REG_GPI_DATA14:
      case REG_GPI_DATA15:
        break;

      default:
        error_report("GPI RD: 0x%08lX --> 0x%08lX", gpi->iomem[1].addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_gpi_write_data(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980GPIState *gpi = opaque;

    switch(addr) {
      case REG_GPI_DATA0:
      case REG_GPI_DATA1:
      case REG_GPI_DATA2:
      case REG_GPI_DATA3:
      case REG_GPI_DATA4:
      case REG_GPI_DATA5:
      case REG_GPI_DATA6:
      case REG_GPI_DATA7:
      case REG_GPI_DATA8:
      case REG_GPI_DATA9:
      case REG_GPI_DATA10:
      case REG_GPI_DATA11:
      case REG_GPI_DATA12:
      case REG_GPI_DATA13:
      case REG_GPI_DATA14:
      case REG_GPI_DATA15:
        break;

      default:
        error_report("GPI WR: 0x%08lX <-- 0x%08lX", gpi->iomem[1].addr + addr, value);
        break;
    }
}

static void nuc980_gpi_instance_init(Object *obj)
{
    NUC980GPIState *gpi = NUC980_GPI(obj);
    
    static const MemoryRegionOps gpi_ops0 = {
      .read       = nuc980_gpi_read_reg,
      .write      = nuc980_gpi_write_reg,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };
    
    static const MemoryRegionOps gpi_ops1 = {
      .read       = nuc980_gpi_read_data,
      .write      = nuc980_gpi_write_data,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&gpi->iomem[0], obj, &gpi_ops0, gpi, "gpi", 0x40);
    memory_region_init_io(&gpi->iomem[1], obj, &gpi_ops1, gpi, "gpi", 0x40);

    sysbus_init_mmio(SYS_BUS_DEVICE(gpi), &gpi->iomem[0]);
    sysbus_init_mmio(SYS_BUS_DEVICE(gpi), &gpi->iomem[1]);
}

static void nuc980_gpi_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 GPI Controller";
    dev_class->reset = NULL;
}

/*------------------------------ GPI TYPE -----------------------------------*/

static const TypeInfo nuc980_gpi_type = {
    .name          = TYPE_NUC980_GPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980GPIState),
    .instance_init = nuc980_gpi_instance_init,
    .class_init    = nuc980_gpi_class_init,
};

/*****************************************************************************/
/*                             Ethernet DRIVER                               */
/*****************************************************************************/

/*---------------------------- ETH MODULE NAME ------------------------------*/

#define TYPE_NUC980_ETH "nuc980-eth"

/*---------------------------- ETH DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980ETHState, NUC980_ETH)

struct NUC980ETHState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq[2];
};

/*----------------------------- ETH FUNCTIONS -------------------------------*/

static uint64_t nuc980_eth_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980ETHState *eth = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        (void)&eth;
        //error_report("ETH RD: 0x%08lX --> 0x%08lX", eth->iomem.addr + addr, ret);
        break;
    }


    return ret;
}

static void nuc980_eth_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980ETHState *eth = opaque;

    switch(addr) {
      default:
        (void)&eth;
        //error_report("ETH WR: 0x%08lX <-- 0x%08lX", eth->iomem.addr + addr, value);
        break;
    }

}

static void nuc980_eth_instance_init(Object *obj)
{
    NUC980ETHState *eth = NUC980_ETH(obj);
    
    static const MemoryRegionOps eth_ops = {
      .read       = nuc980_eth_read,
      .write      = nuc980_eth_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&eth->iomem, obj, &eth_ops, eth, "eth", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(eth), &eth->iomem);
}

static void nuc980_eth_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 ETH Controller";
    dev_class->reset = NULL;
}

/*------------------------------ ETH TYPE -----------------------------------*/

static const TypeInfo nuc980_eth_type = {
    .name          = TYPE_NUC980_ETH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980ETHState),
    .instance_init = nuc980_eth_instance_init,
    .class_init    = nuc980_eth_class_init,
};


/*****************************************************************************/
/*                               USB DRIVER                                  */
/*****************************************************************************/

/*---------------------------- USB MODULE NAME ------------------------------*/

#define TYPE_NUC980_USB "nuc980-usb"

/*---------------------------- USB DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980USBState, NUC980_USB)

struct NUC980USBState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem[2];
    qemu_irq      irq[2];
};

/*----------------------------- USB FUNCTIONS -------------------------------*/

static uint64_t nuc980_usb_read_ehci(void *opaque, hwaddr addr, unsigned size)
{
    NUC980USBState *usb = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        (void)&usb;
        //error_report("USB RD: 0x%08lX --> 0x%08lX", usb->iomem[0].addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_usb_write_ehci(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980USBState *usb = opaque;

    switch(addr) {
      default:
        (void)&usb;
        //error_report("USB WR: 0x%08lX <-- 0x%08lX", usb->iomem[0].addr + addr, value);
        break;
    }
}

static uint64_t nuc980_usb_read_ohci(void *opaque, hwaddr addr, unsigned size)
{
    NUC980USBState *usb = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        (void)&usb;
        //error_report("USB RD: 0x%08lX --> 0x%08lX", usb->iomem[1].addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_usb_write_ohci(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980USBState *usb = opaque;

    switch(addr) {
      default:
        (void)&usb;
        //error_report("USB WR: 0x%08lX <-- 0x%08lX", usb->iomem[1].addr + addr, value);
        break;
    }
}

static void nuc980_usb_instance_init(Object *obj)
{
    NUC980USBState *usb = NUC980_USB(obj);
    
    static const MemoryRegionOps usb_ops0 = {
      .read       = nuc980_usb_read_ehci,
      .write      = nuc980_usb_write_ehci,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };
    
    static const MemoryRegionOps usb_ops1 = {
      .read       = nuc980_usb_read_ohci,
      .write      = nuc980_usb_write_ohci,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&usb->iomem[0], obj, &usb_ops0, usb, "usb", 0x1000);
    memory_region_init_io(&usb->iomem[1], obj, &usb_ops1, usb, "usb", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(usb), &usb->iomem[0]);
    sysbus_init_mmio(SYS_BUS_DEVICE(usb), &usb->iomem[1]);
}

static void nuc980_usb_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 USB Controller";
    dev_class->reset = NULL;
}

/*------------------------------ USB TYPE -----------------------------------*/

static const TypeInfo nuc980_usb_type = {
    .name          = TYPE_NUC980_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980USBState),
    .instance_init = nuc980_usb_instance_init,
    .class_init    = nuc980_usb_class_init,
};

/*****************************************************************************/
/*                         INTERNAL NAND FLASH DRIVER                        */
/*****************************************************************************/

/*---------------------------- FMI MODULE NAME ------------------------------*/

#define TYPE_NUC980_FMI "nuc980-fmi"

/*----------------------------- FMI REGISTERS -------------------------------*/

#define FMI_DMACTL       0x400
#define FMI_DMASA        0x408
#define FMI_DMAABCNT     0x40C
#define FMI_DMAINTEN     0x410
#define FMI_DMAINTSTS    0x414
#define FMI_GCTL         0x800
#define FMI_NANDCTL      0x8A0
#define FMI_NANDTMCTL    0x8A4
#define FMI_NANDINTEN    0x8A8
#define FMI_NANDINTSTS   0x8AC
#define FMI_NANDCMD      0x8B0
#define FMI_NANDADDR     0x8B4
#define FMI_NANDDATA     0x8B8
#define FMI_NANDRACTL    0x8BC
#define FMI_NANDECTL     0x8C0
#define FMI_NANDRA0      0xA00
#define FMI_NANDRA1      0xA04
#define FMI_NANDRA2      0xA08
#define FMI_NANDRA3      0xA0C
#define FMI_NANDRA4      0xA10
#define FMI_NANDRA5      0xA14
#define FMI_NANDRA6      0xA18
#define FMI_NANDRA7      0xA1C
#define FMI_NANDRA8      0xA20
#define FMI_NANDRA9      0xA24
#define FMI_NANDRA10     0xA28
#define FMI_NANDRA11     0xA2C
#define FMI_NANDRA12     0xA30
#define FMI_NANDRA13     0xA34
#define FMI_NANDRA14     0xA38
#define FMI_NANDRA15     0xA3C

/*---------------------------- FMI DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980FMIState, NUC980_FMI)

struct NUC980FMIState {
    SysBusDevice    parent_obj;
    MemoryRegion    iomem;
    qemu_irq        irq;
    uint32_t        dma_enable;
    uint32_t        dma_addr;
    uint32_t        nand_ctrl;
    uint32_t        nand_tmctl;
    uint32_t        int_en;
    uint32_t        int_st;
    uint8_t         cmd;
    uint8_t         addr[4];
    uint8_t         addr_idx;
    uint8_t         data_idx;
    uint32_t        redu_area[16];
};

/*----------------------------- FMI FUNCTIONS -------------------------------*/

static void nuc980_fmi_dma(NUC980FMIState *fmi)
{
    uint32_t sec_size;
    uint32_t src_addr;
    uint32_t dst_addr;

    sec_size = 0x800;
    src_addr = ((fmi->addr[3]<<8) | (fmi->addr[2]<<0))*sec_size;
    dst_addr = fmi->dma_addr;

    cpu_physical_memory_write(dst_addr, nuc980_boot_flash+src_addr, sec_size);
}

static uint64_t nuc980_fmi_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980FMIState *fmi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case FMI_DMACTL:
        ret = fmi->dma_enable;
        break;

      case FMI_DMASA:
        ret = fmi->dma_addr;
        break;

      case FMI_DMAABCNT:
        break;

      case FMI_DMAINTEN:
        break;

      case FMI_DMAINTSTS:
        break;

      case FMI_GCTL:
        break;

      case FMI_NANDCTL:
        ret = fmi->nand_ctrl;
        break;

      case FMI_NANDTMCTL:
        ret = fmi->nand_tmctl;
        break;

      case FMI_NANDINTEN:
        ret = fmi->int_en;
        break;

      case FMI_NANDINTSTS:
        ret = fmi->int_st;
        break;

      case FMI_NANDCMD:
        break;

      case FMI_NANDADDR:
        break;

      case FMI_NANDDATA:
        /* process command here */
        if (fmi->cmd == 0x90) {
          /* READ ID (MT29F2G08AAD) */
          if (fmi->data_idx == 0) {
            ret = 0x2C;
          } else if (fmi->data_idx == 1) {
            ret = 0xDA;
          } else if (fmi->data_idx == 2) {
            ret = 0x80;
          } else if (fmi->data_idx == 3) {
            ret = 0x95;
          } else if (fmi->data_idx == 4) {
            ret = 0x50;
          } else {
            ret = 0xFF;
          }
        } else if (fmi->cmd == 0x30) {
          /* START READ */
          if (fmi->addr[0] != 0 || fmi->addr[1] != 0) {
            ret = 0xFF;
          }
        }
        fmi->data_idx++;
        break;

      case FMI_NANDRA0:
      case FMI_NANDRA1:
      case FMI_NANDRA2:
      case FMI_NANDRA3:
      case FMI_NANDRA4:
      case FMI_NANDRA5:
      case FMI_NANDRA6:
      case FMI_NANDRA7:
      case FMI_NANDRA8:
      case FMI_NANDRA9:
      case FMI_NANDRA10:
      case FMI_NANDRA11:
      case FMI_NANDRA12:
      case FMI_NANDRA13:
      case FMI_NANDRA14:
      case FMI_NANDRA15:
        ret = fmi->redu_area[(addr&0xFF)>>2];
        break;

      default:
        error_report("FMI RD: 0x%08lX --> 0x%08lX", fmi->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_fmi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980FMIState *fmi = opaque;

    switch(addr) {
      case FMI_DMACTL:
        fmi->dma_enable = value & 1;
        break;

      case FMI_DMASA:
        fmi->dma_addr = value;
        break;

      case FMI_DMAABCNT:
        break;

      case FMI_DMAINTEN:
        break;

      case FMI_DMAINTSTS:
        break;

      case FMI_GCTL:
        break;

      case FMI_NANDCTL:
        if (value & 0) {
          /* software reset */
        }
        if (value & 2) {
          /* DMA read */
          nuc980_fmi_dma(fmi);
          fmi->int_st |= (1<<0);
        }
        if (value & 4) {
          /* DMA write */
        }
        if (value & 8) {
          /* redundant area read */
        }
        fmi->nand_ctrl = value & 0xFFFFFFF0;
        break;

      case FMI_NANDTMCTL:
        fmi->nand_tmctl = value;
        break;

      case FMI_NANDINTEN:
        fmi->int_en = value;
        break;

      case FMI_NANDINTSTS:
        if (value & (1<<0)) {
          fmi->int_st &= ~(1<<0);
        }
        if (value & (1<<2)) {
          fmi->int_st &= ~(1<<2);
        }
        if (value & (1<<10)) {
          fmi->int_st &= ~(1<<10);
        }
        break;

      case FMI_NANDCMD:
        fmi->cmd = value;
        fmi->addr_idx = 0;
        fmi->data_idx = 0;
        break;

      case FMI_NANDADDR:
        if (fmi->addr_idx < 4) {
          fmi->addr[fmi->addr_idx] = value;
        }    
        fmi->addr_idx++;
        break;

      case FMI_NANDDATA:
        fmi->data_idx++;
        break;

      case FMI_NANDRACTL:
        break;

      case FMI_NANDECTL:
        break;

      case FMI_NANDRA0:
      case FMI_NANDRA1:
      case FMI_NANDRA2:
      case FMI_NANDRA3:
      case FMI_NANDRA4:
      case FMI_NANDRA5:
      case FMI_NANDRA6:
      case FMI_NANDRA7:
      case FMI_NANDRA8:
      case FMI_NANDRA9:
      case FMI_NANDRA10:
      case FMI_NANDRA11:
      case FMI_NANDRA12:
      case FMI_NANDRA13:
      case FMI_NANDRA14:
      case FMI_NANDRA15:
        fmi->redu_area[(addr&0xFF)>>2] = value;
        break;

      default:
        error_report("FMI WR: 0x%08lX <-- 0x%08lX", fmi->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_fmi_instance_init(Object *obj)
{
    NUC980FMIState *fmi = NUC980_FMI(obj);
    
    static const MemoryRegionOps fmi_ops = {
      .read       = nuc980_fmi_read,
      .write      = nuc980_fmi_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&fmi->iomem, obj, &fmi_ops, fmi, "fmi", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(fmi), &fmi->iomem);
    
    fmi->nand_ctrl = 0x02860090;
    fmi->int_st    = (1<<18);
}

static void nuc980_fmi_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 FMI Controller";
    dev_class->reset = NULL;
}

/*------------------------------ FMI TYPE -----------------------------------*/

static const TypeInfo nuc980_fmi_type = {
    .name          = TYPE_NUC980_FMI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980FMIState),
    .instance_init = nuc980_fmi_instance_init,
    .class_init    = nuc980_fmi_class_init,
};

/*****************************************************************************/
/*                              RTC CONTROLLER                               */
/*****************************************************************************/

/*---------------------------- RTC MODULE NAME ------------------------------*/

#define TYPE_NUC980_RTC "nuc980-rtc"

/*----------------------------- RTC REGISTERS -------------------------------*/

#define REG_RTC_INIT       0x000
#define REG_RTC_RWEN       0x004
#define REG_RTC_FREQADJ    0x008
#define REG_RTC_TIME       0x00C
#define REG_RTC_CAL        0x010
#define REG_RTC_TIMEFMT    0x014
#define REG_RTC_WEEKDAY    0x018
#define REG_RTC_TALM       0x01C
#define REG_RTC_CALM       0x020
#define REG_RTC_LEAPYEAR   0x024
#define REG_RTC_INTEN      0x028
#define REG_RTC_INTSTS     0x02C
#define REG_RTC_TICK       0x030
#define REG_RTC_PWRCTL     0x034
#define REG_RTC_PWRCNT     0x038
#define REG_RTC_CLKCTL     0x03C
#define REG_RTC_SPR0       0x040 
#define REG_RTC_SPR1       0x044
#define REG_RTC_SPR2       0x048
#define REG_RTC_SPR3       0x04C
#define REG_RTC_SPR4       0x050
#define REG_RTC_SPR5       0x054
#define REG_RTC_SPR6       0x058
#define REG_RTC_SPR7       0x05C
#define REG_RTC_SPR8       0x060 
#define REG_RTC_SPR9       0x064
#define REG_RTC_SPR10      0x068
#define REG_RTC_SPR11      0x06C
#define REG_RTC_SPR12      0x070
#define REG_RTC_SPR13      0x074
#define REG_RTC_SPR14      0x078
#define REG_RTC_SPR15      0x07C

/*---------------------------- RTC DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980RTCState, NUC980_RTC)

struct NUC980RTCState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    QEMUTimer    *ts;
    qemu_irq      irq;
    uint32_t      tick_cnt;
};

/*----------------------------- RTC FUNCTIONS -------------------------------*/

static void nuc980_rtc_cb(void *opaque)
{
    NUC980RTCState *rtc = opaque;
#if 0
    if (rtc->tick_cnt == 100) {
      //qemu_set_irq(rtc->irq, 1);
      rtc->tick_cnt = 0;
    }
    
    rtc->tick_cnt++;
    
    if (rtc->tick_cnt == 10) {
      //qemu_set_irq(rtc->irq, 0);
    }
#endif
    timer_mod_ns(rtc->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+100000);
}


static uint64_t nuc980_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980RTCState *rtc = opaque;
    uint64_t        ret = 0;

    switch (addr) {
      default:
        error_report("RTC RD: 0x%08lX --> 0x%08lX", rtc->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_rtc_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980RTCState *rtc = opaque;

    switch (addr) {
      default:
        error_report("RTC WR: 0x%08lX <-- 0x%08lX", rtc->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_rtc_instance_init(Object *obj)
{
    NUC980RTCState *rtc = NUC980_RTC(obj);
    
    static const MemoryRegionOps rtc_ops = {
      .read       = nuc980_rtc_read,
      .write      = nuc980_rtc_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&rtc->iomem, obj, &rtc_ops, rtc, "rtc", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(rtc), &rtc->iomem);

    rtc->ts = timer_new_ns(QEMU_CLOCK_REALTIME, nuc980_rtc_cb, rtc);
    timer_mod_anticipate_ns(rtc->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME));
}

static void nuc980_rtc_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 RTC Controller";
    dev_class->reset = NULL;
}

/*------------------------------ RTC TYPE -----------------------------------*/

static const TypeInfo nuc980_rtc_type = {
    .name          = TYPE_NUC980_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980RTCState),
    .instance_init = nuc980_rtc_instance_init,
    .class_init    = nuc980_rtc_class_init,
};

/*****************************************************************************/
/*                               TIMER DRIVER                                */
/*****************************************************************************/

/*----------------------------- TMR MODULE NAME -----------------------------*/

#define TYPE_NUC980_TMR "nuc980-tmr"

/*----------------------------- TMR REGISTERS -------------------------------*/

#define REG_TMR_CTL     0x000
#define REG_TMR_PRECNT  0x004
#define REG_TMR_CMP     0x008
#define REG_TMR_INTEN   0x00C
#define REG_TMR_INTSTS  0x010
#define REG_TMR_CNT     0x014
#define REG_TMR_CAP     0x018
#define REG_TMR_ECTL    0x020

/*---------------------------- TMR DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980TMRState, NUC980_TMR)

struct NUC980TMRState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq;
    Clock        *clk;
    QEMUTimer    *ts;
    uint32_t      ctl;
    uint32_t      sts;
    uint64_t      ctr;
    uint64_t      cmp;
    uint32_t      pre;
    uint32_t      ien;
};

/*----------------------------- TMR FUNCTIONS -------------------------------*/

static void nuc980_tmr_cb(void *opaque)
{
    NUC980TMRState *tmr    = opaque;
    uint64_t        period = 0;

    /* COUNTING */
    if (tmr->ctl & 1) {
      tmr->ctr+=1000;
      if (tmr->ctr >= tmr->cmp) {
        tmr->ctr = tmr->cmp;
      }
      //if (tmr->cmp < 0xFFFFF)
      //  printf("counter: 0x%016lX 0x%016lX 0x%02X 0x%02X\n", tmr->ctr, tmr->cmp, tmr->ctl, tmr->ien);
      if (tmr->ctr == tmr->cmp) {
        //printf("COUNTED %d!\n", tmr->ien);
        if (((tmr->ctl>>4)&3) == 0) {
          tmr->ctl &= ~3;
        }
        tmr->ctr = 0;
        tmr->sts |= 1;
        if (tmr->ien & 1) {
          qemu_set_irq(tmr->irq, 1);
        }
      }
    }
    
    if (clock_get_hz(tmr->clk) == 0) {
      period = 1000;
    } else {
      period = 1000000000UL / clock_get_hz(tmr->clk);
      period *= (tmr->pre&0xFF)+1;
      period *= 1000;
    }

    timer_mod(tmr->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+period);
}

static uint64_t nuc980_tmr_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980TMRState *tmr = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_TMR_CTL:
        ret = tmr->ctl;
        break;

      case REG_TMR_CMP:
        ret = tmr->cmp;
        break;

      case REG_TMR_INTSTS:
        ret = tmr->sts;
        break;

      case REG_TMR_CNT:
        ret = tmr->ctr;
        break;

      default:
        error_report("TMR RD: 0x%08lX --> 0x%08lX", tmr->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_tmr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980TMRState *tmr = opaque;

    switch(addr) {
      case REG_TMR_CTL:
        //printf("COUNT FOR %ld %lX %ld\n", tmr->cmp, value, tmr->ctr);
        tmr->ctl = value;
        if (tmr->ctl & 2) {
          tmr->ctr = 0;
          tmr->ctl &= ~2;
          tmr->ctl |= 1;
        }
        break;

      case REG_TMR_PRECNT:
        tmr->pre = value;
        break;

      case REG_TMR_INTEN:
        tmr->ien = value;
        break;

      case REG_TMR_CMP:
        tmr->ctl &= ~3;
        tmr->cmp = (value) & 0xFFFFFF;
        tmr->ctr = 0;
        tmr->ctl |= 2;
        break;

      case REG_TMR_INTSTS:
        if ((value & 1) && (tmr->sts & 1)) {
          tmr->sts &= ~(1);
          qemu_set_irq(tmr->irq, 0);
        }
        break;

      default:
        error_report("TMR WR: 0x%08lX <-- 0x%08lX", tmr->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_tmr_instance_init(Object *obj)
{
    NUC980TMRState *tmr = NUC980_TMR(obj);
    
    static const MemoryRegionOps tmr_ops = {
      .read       = nuc980_tmr_read,
      .write      = nuc980_tmr_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&tmr->iomem, obj, &tmr_ops, tmr, "tmr", 0x100);

    sysbus_init_mmio(SYS_BUS_DEVICE(tmr), &tmr->iomem);
    
    tmr->ts = timer_new_ns(QEMU_CLOCK_REALTIME, nuc980_tmr_cb, tmr);
    timer_mod_anticipate_ns(tmr->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME));
}

static void nuc980_tmr_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 TMR Controller";
    dev_class->reset = NULL;
}

/*------------------------------ TMR TYPE -----------------------------------*/

static const TypeInfo nuc980_tmr_type = {
    .name          = TYPE_NUC980_TMR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980TMRState),
    .instance_init = nuc980_tmr_instance_init,
    .class_init    = nuc980_tmr_class_init,
};

/*****************************************************************************/
/*                               SPI DRIVER                                  */
/*****************************************************************************/

/*--------------------------- SPI MODULE NAME -------------------------------*/

#define TYPE_NUC980_SPI "nuc980-spi"

/*---------------------------- SPI DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SPIState, NUC980_SPI)

struct NUC980SPIState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    SSIBus       *bus;
    uint32_t      lol_reg;
};

/*----------------------------- SPI FUNCTIONS -------------------------------*/

static uint64_t nuc980_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SPIState *spi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        error_report("SPI RD: 0x%08lX --> 0x%08lX", spi->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SPIState *spi = opaque;

    switch(addr) {
      default:
        error_report("SPI WR: 0x%08lX <-- 0x%08lX", spi->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_spi_instance_init(Object *obj)
{
    NUC980SPIState *spi = NUC980_SPI(obj);
    
    static const MemoryRegionOps spi_ops = {
      .read       = nuc980_spi_read,
      .write      = nuc980_spi_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&spi->iomem, obj, &spi_ops, spi, "spi", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(spi), &spi->iomem);

    spi->bus = ssi_create_bus(DEVICE(spi), "ssi");
}

static void nuc980_spi_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SPI Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SPI TYPE -----------------------------------*/

static const TypeInfo nuc980_spi_type = {
    .name          = TYPE_NUC980_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SPIState),
    .instance_init = nuc980_spi_instance_init,
    .class_init    = nuc980_spi_class_init,
};

/*****************************************************************************/
/*                               UART DRIVER                                 */
/*****************************************************************************/

/*----------------------------- SER MODULE NAME -----------------------------*/

#define TYPE_NUC980_SER "nuc980-ser"

/*------------------------------ SER REGISTERS ------------------------------*/

#define REG_SER_DAT        0x00
#define REG_SER_INTEN      0x04
#define REG_SER_FIFO       0x08
#define REG_SER_LINE       0x0C
#define REG_SER_MODEM      0x10
#define REG_SER_MODEMSTS   0x14
#define REG_SER_FIFOSTS    0x18
#define REG_SER_INTSTS     0x1C
#define REG_SER_TOUT       0x20
#define REG_SER_BAUD       0x24
#define REG_SER_IRDA       0x28
#define REG_SER_ALTCTL     0x2C
#define REG_SER_FUNCSEL    0x30
#define REG_SER_LINCTL     0x34
#define REG_SER_LINSTS     0x38
#define REG_SER_BRCOMP     0x3C
#define REG_SER_WKCTL      0x40
#define REG_SER_WKSTS      0x44
#define REG_SER_DWKCOMP    0x48

/*---------------------------- SER DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SERState, NUC980_SER)

struct NUC980SERState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    CharBackend   be;
    bool          rx_avail;
    uint8_t       rx_data;
    qemu_irq      irq;
    uint32_t      ien;
    uint32_t      fifo_ctl;
    uint32_t      line_ctl;
    uint32_t      modm_ctl;
    uint32_t      fifo_sts;
    uint32_t      modm_sts;
    uint32_t      intr_sts;
    uint32_t      baud;
};

/*----------------------------- SER FUNCTIONS -------------------------------*/

static int nuc980_ser_chardev_canrd(void *opaque)
{
    NUC980SERState *ser = opaque;

    return (ser->rx_avail == false);
}

static void nuc980_ser_chardev_read(void *opaque, const uint8_t *buf, int size)
{
    NUC980SERState *ser = opaque;

    ser->rx_avail = true;
    ser->rx_data  = buf[0];

    if (ser->ien & 1) {
      qemu_set_irq(ser->irq, 1);
      ser->intr_sts |= 1;
    }
}

static void nuc980_ser_chardev_write(void *opaque, const uint8_t *buf, int size)
{
    NUC980SERState *ser = opaque;

    qemu_chr_fe_write(&ser->be, buf, size);
}

static void nuc980_ser_chardev_event(void *opaque, QEMUChrEvent event)
{
}

static int nuc980_ser_chardev_canchg(void *opaque)
{
    return 1;
}

static void nuc980_ser_chardev_attach(NUC980SERState *ser, Chardev *s) {
    qemu_chr_fe_init(&ser->be, s, &error_abort);
    qemu_chr_fe_set_handlers(&ser->be,
                             nuc980_ser_chardev_canrd,
                             nuc980_ser_chardev_read,
                             nuc980_ser_chardev_event,
                             nuc980_ser_chardev_canchg,
                             ser,
                             NULL,
                             false);
}

static uint64_t nuc980_ser_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SERState *ser = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_SER_DAT:
        ret = ser->rx_data;
        ser->rx_avail = false;
        if (ser->intr_sts & 1) {
          ser->intr_sts &= ~1;
          qemu_set_irq(ser->irq, 0);
        }
        break;
        
      case REG_SER_INTEN:
        ret = ser->ien;
        break;
        
      case REG_SER_FIFO:
        ret = ser->fifo_ctl;
        break;
        
      case REG_SER_LINE:
        ret = ser->line_ctl;
        break;
        
      case REG_SER_MODEM:
        ret = ser->modm_ctl;
        break;
        
      case REG_SER_MODEMSTS:
        ret = ser->modm_sts;
        break;

      case REG_SER_FIFOSTS:
        ret |= (1<<28); // TXEMPTF
        ret |= (1<<22); // TXEMPTY
        if (!ser->rx_avail) {
          ret |= (1<<14); // RXEMPTY
        }
        // FIXME: there is a possible bug in the NUC980 UART Linux
        //        device driver (drivers/tty/serial/nuc980_serial.c)
        //        in function nuc980serial_start_tx(). By the end
        //        of the function, the driver uses this if test:
        //           uart_circ_chars_pending(xmit)<(16-((serial_in(up, UART_REG_FSR)>>16)&0x3F)
        //        which, uart_circ_chars_pending() is too big, 
        //        needs bits 4&7 of TXPTR to be set to 1, so that 
        //        serial_in(up, UART_REG_FSR)>>16)&0x3F evaluates to 0x30 or 0x20
        //        at least, and then 16-0x30 will yield in a negative value treated
        //        as unsugned, then uart_circ_chars_pending(xmit) will work
        //        in that case.
        ret |= 0x300000;
        break;

      case REG_SER_INTSTS:
        ret = ser->intr_sts;
        break;

      case REG_SER_TOUT:
        break;

      case REG_SER_BAUD:
        ret = ser->baud;
        break;

      default:
        error_report("SER RD: 0x%08lX --> 0x%08lX", ser->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_ser_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SERState *ser = opaque;

    switch(addr) {
      case REG_SER_DAT:
        nuc980_ser_chardev_write(ser, (const uint8_t *) &value, 1);
        break;
        
      case REG_SER_INTEN:
        ser->ien = value;
        break;
        
      case REG_SER_FIFO:
        ser->fifo_ctl = value;
        break;
        
      case REG_SER_LINE:
        ser->line_ctl = value;
        break;
        
      case REG_SER_MODEM:
        ser->modm_ctl = value;
        break;
        
      case REG_SER_MODEMSTS:
        //ser->modm_sts = value;
        break;
        
      case REG_SER_FIFOSTS:
        //ser->fifo_sts = value;
        break;

      case REG_SER_INTSTS:
        ser->intr_sts &= ~value;
        break;

      case REG_SER_TOUT:
        break;

      case REG_SER_BAUD:
        ser->baud = value;
        break;

      default:
        error_report("SER WR: 0x%08lX <-- 0x%08lX", ser->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_ser_instance_init(Object *obj)
{
    NUC980SERState *ser = NUC980_SER(obj);
    
    static const MemoryRegionOps ser_ops = {
      .read       = nuc980_ser_read,
      .write      = nuc980_ser_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&ser->iomem, obj, &ser_ops, ser, "ser", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(ser), &ser->iomem);
}

static void nuc980_ser_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SER Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SER TYPE -----------------------------------*/

static const TypeInfo nuc980_ser_type = {
    .name          = TYPE_NUC980_SER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SERState),
    .instance_init = nuc980_ser_instance_init,
    .class_init    = nuc980_ser_class_init,
};

/*****************************************************************************/
/*                               PDMA DRIVER                                 */
/*****************************************************************************/

/*---------------------------- DMA MODULE NAME ------------------------------*/

#define TYPE_NUC980_DMA "nuc980-dma"

/*---------------------------- DMA DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980DMAState, NUC980_DMA)

struct NUC980DMAState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq[2];
};

/*----------------------------- DMA FUNCTIONS -------------------------------*/

static uint64_t nuc980_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980DMAState *dma = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        break;
    }
        error_report("DMA RD: 0x%08lX --> 0x%08lX", dma->iomem.addr + addr, ret);

    return ret;
}

static void nuc980_dma_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980DMAState *dma = opaque;

    switch(addr) {

      default:
        break;
    }
        error_report("DMA WR: 0x%08lX <-- 0x%08lX", dma->iomem.addr + addr, value);
}

static void nuc980_dma_instance_init(Object *obj)
{
    NUC980DMAState *dma = NUC980_DMA(obj);
    
    static const MemoryRegionOps dma_ops = {
      .read       = nuc980_dma_read,
      .write      = nuc980_dma_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&dma->iomem, obj, &dma_ops, dma, "dma", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(dma), &dma->iomem);
}

static void nuc980_dma_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 DMA Controller";
    dev_class->reset = NULL;
}

/*------------------------------ DMA TYPE -----------------------------------*/

static const TypeInfo nuc980_dma_type = {
    .name          = TYPE_NUC980_DMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980DMAState),
    .instance_init = nuc980_dma_instance_init,
    .class_init    = nuc980_dma_class_init,
};

/*****************************************************************************/
/*                                BOARD DRIVER                               */
/*****************************************************************************/

/*----------------------------- SOC MODULE NAME -----------------------------*/

#define TYPE_NUC980_SOC      MACHINE_TYPE_NAME("nuc980-soc")

/*------------------------------ SOC FUNCTIONS ------------------------------*/

static void nuc980_soc_ldflash(MachineState *machine)
{
    FILE *file_desc = NULL;
    char *file_name = NULL;
    char  read_cnt  = 0;

    if (!machine->firmware) {
      error_report("No ROM boot image provided.");
      exit(1);
    }
    
    file_name = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!file_name) {
      error_report("Could not find ROM image '%s'", machine->firmware);
      exit(1);
    }
    
    file_desc = fopen(file_name, "r");

    read_cnt = fread(nuc980_boot_flash, sizeof(nuc980_boot_flash), 1, file_desc);
    if (read_cnt != 1) {
      error_report("Failed to read from '%s'", machine->firmware);
      exit(1);
    }

    fclose(file_desc);
}

static void nuc980_soc_reset(MachineState *machine)
{
    /* on real board there is a 16.5KB internal boot ROM (IBR)
     * that initializes the 64MB SDRAM, maps it to address 0x00000 
     * and loads early-stage SPL loader from flash into RAM.
     */
    int spl_addr   = 0x00200;
    int spl_size   = 0x10000;

    cpu_physical_memory_write(spl_addr, nuc980_boot_flash, spl_size);

    /* set CPU to jump to SPL code */
    //cpu_set_pc(CPU(cpu), 0x200);
}

static void nuc980_soc_instance_init(MachineState *machine)
{
    ARMCPU         *cpu       = NULL;
    MemoryRegion   *mem       = get_system_memory();
    qemu_irq        irq[64]   = {0};
    NUC980SYSState *sys       = NULL;
    NUC980CLKState *clk       = NULL;
    NUC980SDRState *sdr       = NULL;
    NUC980ETHState *eth[2]    = {0};
    NUC980USBState *usb       = NULL;
    NUC980FMIState *fmi       = NULL;
    NUC980PICState *pic       = NULL;
    NUC980GPIState *gpi[7]    = {0};
    NUC980RTCState *rtc       = NULL;
    NUC980TMRState *tmr[6]    = {0};
    NUC980SPIState *spi[3]    = {0};
    NUC980SERState *ser[10]   = {0};
    NUC980DMAState *dma       = NULL;

    /* allocate new CPU */
    cpu = ARM_CPU(object_new("arm926-arm-cpu"));
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);

    /* SYS Controller */
    sys = NUC980_SYS(object_new(TYPE_NUC980_SYS));
    memory_region_add_subregion(mem, 0xB0000000, &sys->iomem);

    /* CLK Controller */
    clk = NUC980_CLK(object_new(TYPE_NUC980_CLK));
    memory_region_add_subregion(mem, 0xB0000200, &clk->iomem);

    /* SDR Controller */
    sdr = NUC980_SDR(object_new(TYPE_NUC980_SDR));
    memory_region_add_subregion(mem, 0x00000000, &sdr->sdram);
    memory_region_add_subregion(mem, 0xBC000000, &sdr->bootram);
    memory_region_add_subregion(mem, 0xB0002000, &sdr->iomem);

    /* PIC */
    pic = NUC980_PIC(object_new(TYPE_NUC980_PIC));
    memory_region_add_subregion(mem, 0xB0042000, &pic->iomem);
    irq[ 0] = qdev_get_gpio_in(DEVICE(pic),  0);
    irq[ 1] = qdev_get_gpio_in(DEVICE(pic),  1);
    irq[ 2] = qdev_get_gpio_in(DEVICE(pic),  2);
    irq[ 3] = qdev_get_gpio_in(DEVICE(pic),  3);
    irq[ 4] = qdev_get_gpio_in(DEVICE(pic),  4);
    irq[ 5] = qdev_get_gpio_in(DEVICE(pic),  5);
    irq[ 6] = qdev_get_gpio_in(DEVICE(pic),  6);
    irq[ 7] = qdev_get_gpio_in(DEVICE(pic),  7);
    irq[ 8] = qdev_get_gpio_in(DEVICE(pic),  8);
    irq[ 9] = qdev_get_gpio_in(DEVICE(pic),  9);
    irq[10] = qdev_get_gpio_in(DEVICE(pic), 10);
    irq[11] = qdev_get_gpio_in(DEVICE(pic), 11);
    irq[12] = qdev_get_gpio_in(DEVICE(pic), 12);
    irq[13] = qdev_get_gpio_in(DEVICE(pic), 13);
    irq[14] = qdev_get_gpio_in(DEVICE(pic), 14);
    irq[15] = qdev_get_gpio_in(DEVICE(pic), 15);
    irq[16] = qdev_get_gpio_in(DEVICE(pic), 16);
    irq[17] = qdev_get_gpio_in(DEVICE(pic), 17);
    irq[18] = qdev_get_gpio_in(DEVICE(pic), 18);
    irq[19] = qdev_get_gpio_in(DEVICE(pic), 19);
    irq[20] = qdev_get_gpio_in(DEVICE(pic), 20);
    irq[21] = qdev_get_gpio_in(DEVICE(pic), 21);
    irq[22] = qdev_get_gpio_in(DEVICE(pic), 22);
    irq[23] = qdev_get_gpio_in(DEVICE(pic), 23);
    irq[24] = qdev_get_gpio_in(DEVICE(pic), 24);
    irq[25] = qdev_get_gpio_in(DEVICE(pic), 25);
    irq[26] = qdev_get_gpio_in(DEVICE(pic), 26);
    irq[27] = qdev_get_gpio_in(DEVICE(pic), 27);
    irq[28] = qdev_get_gpio_in(DEVICE(pic), 28);
    irq[29] = qdev_get_gpio_in(DEVICE(pic), 29);
    irq[30] = qdev_get_gpio_in(DEVICE(pic), 30);
    irq[31] = qdev_get_gpio_in(DEVICE(pic), 31);
    irq[32] = qdev_get_gpio_in(DEVICE(pic), 32);
    irq[33] = qdev_get_gpio_in(DEVICE(pic), 33);
    irq[34] = qdev_get_gpio_in(DEVICE(pic), 34);
    irq[35] = qdev_get_gpio_in(DEVICE(pic), 35);
    irq[36] = qdev_get_gpio_in(DEVICE(pic), 36);
    irq[37] = qdev_get_gpio_in(DEVICE(pic), 37);
    irq[38] = qdev_get_gpio_in(DEVICE(pic), 38);
    irq[39] = qdev_get_gpio_in(DEVICE(pic), 39);
    irq[40] = qdev_get_gpio_in(DEVICE(pic), 40);
    irq[41] = qdev_get_gpio_in(DEVICE(pic), 41);
    irq[42] = qdev_get_gpio_in(DEVICE(pic), 42);
    irq[43] = qdev_get_gpio_in(DEVICE(pic), 43);
    irq[44] = qdev_get_gpio_in(DEVICE(pic), 44);
    irq[45] = qdev_get_gpio_in(DEVICE(pic), 45);
    irq[46] = qdev_get_gpio_in(DEVICE(pic), 46);
    irq[47] = qdev_get_gpio_in(DEVICE(pic), 47);
    irq[48] = qdev_get_gpio_in(DEVICE(pic), 48);
    irq[49] = qdev_get_gpio_in(DEVICE(pic), 49);
    irq[50] = qdev_get_gpio_in(DEVICE(pic), 50);
    irq[51] = qdev_get_gpio_in(DEVICE(pic), 51);
    irq[52] = qdev_get_gpio_in(DEVICE(pic), 52);
    irq[53] = qdev_get_gpio_in(DEVICE(pic), 53);
    irq[54] = qdev_get_gpio_in(DEVICE(pic), 54);
    irq[55] = qdev_get_gpio_in(DEVICE(pic), 55);
    irq[56] = qdev_get_gpio_in(DEVICE(pic), 56);
    irq[57] = qdev_get_gpio_in(DEVICE(pic), 57);
    irq[58] = qdev_get_gpio_in(DEVICE(pic), 58);
    irq[59] = qdev_get_gpio_in(DEVICE(pic), 59);
    irq[60] = qdev_get_gpio_in(DEVICE(pic), 60);
    irq[61] = qdev_get_gpio_in(DEVICE(pic), 61);
    irq[62] = qdev_get_gpio_in(DEVICE(pic), 62);
    irq[63] = qdev_get_gpio_in(DEVICE(pic), 63);
    qdev_connect_gpio_out(DEVICE(pic), 0, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    qdev_connect_gpio_out(DEVICE(pic), 1, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    /* GPIO Controller */
    gpi[0] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[1] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[2] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[3] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[4] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[5] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    gpi[6] = NUC980_GPI(object_new(TYPE_NUC980_GPI));
    memory_region_add_subregion(mem, 0xB0004000, &gpi[0]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004040, &gpi[1]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004080, &gpi[2]->iomem[0]);
    memory_region_add_subregion(mem, 0xB00040C0, &gpi[3]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004100, &gpi[4]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004140, &gpi[5]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004180, &gpi[6]->iomem[0]);
    memory_region_add_subregion(mem, 0xB0004800, &gpi[0]->iomem[1]);
    memory_region_add_subregion(mem, 0xB0004840, &gpi[1]->iomem[1]);
    memory_region_add_subregion(mem, 0xB0004880, &gpi[2]->iomem[1]);
    memory_region_add_subregion(mem, 0xB00048C0, &gpi[3]->iomem[1]);
    memory_region_add_subregion(mem, 0xB0004900, &gpi[4]->iomem[1]);
    memory_region_add_subregion(mem, 0xB0004940, &gpi[5]->iomem[1]);
    memory_region_add_subregion(mem, 0xB0004980, &gpi[6]->iomem[1]);
    gpi[0]->irq = irq[8];
    gpi[1]->irq = irq[9];
    gpi[2]->irq = irq[10];
    gpi[3]->irq = irq[11];
    gpi[4]->irq = irq[49];
    gpi[5]->irq = irq[57];
    gpi[6]->irq = irq[63];

    /* Ethernet */
    eth[0] = NUC980_ETH(object_new(TYPE_NUC980_ETH));
    eth[1] = NUC980_ETH(object_new(TYPE_NUC980_ETH));
    memory_region_add_subregion(mem, 0xB0012000, &eth[0]->iomem);
    memory_region_add_subregion(mem, 0xB0022000, &eth[1]->iomem);
    eth[0]->irq[0] = irq[19];
    eth[1]->irq[1] = irq[20];
    eth[0]->irq[0] = irq[21];
    eth[1]->irq[1] = irq[22];

    /* USB */
    usb = NUC980_USB(object_new(TYPE_NUC980_USB));
    memory_region_add_subregion(mem, 0xB0015000, &usb->iomem[0]);
    memory_region_add_subregion(mem, 0xB0017000, &usb->iomem[1]);
    usb->irq[0] = irq[23];
    usb->irq[1] = irq[24];

    /* FMI */
    fmi = NUC980_FMI(object_new(TYPE_NUC980_FMI));
    memory_region_add_subregion(mem, 0xB0019000, &fmi->iomem);
    fmi->irq = irq[28];

    /* RTC Controller */
    rtc = NUC980_RTC(object_new(TYPE_NUC980_RTC));
    memory_region_add_subregion(mem, 0xB0041000, &rtc->iomem);
    rtc->irq = irq[15];

    /* Timers */
    tmr[0] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[1] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[2] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[3] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[4] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[5] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    memory_region_add_subregion(mem, 0xB0050000, &tmr[0]->iomem);
    memory_region_add_subregion(mem, 0xB0050100, &tmr[1]->iomem);
    memory_region_add_subregion(mem, 0xB0051000, &tmr[2]->iomem);
    memory_region_add_subregion(mem, 0xB0051100, &tmr[3]->iomem);
    memory_region_add_subregion(mem, 0xB0052000, &tmr[4]->iomem);
    memory_region_add_subregion(mem, 0xB0052100, &tmr[5]->iomem);
    tmr[0]->irq = irq[16];
    tmr[1]->irq = irq[17];
    tmr[2]->irq = irq[30];
    tmr[3]->irq = irq[31];
    tmr[4]->irq = irq[32];
    tmr[5]->irq = irq[34];
    tmr[0]->clk = clk->clk_timer[0];
    tmr[1]->clk = clk->clk_timer[1];
    tmr[2]->clk = clk->clk_timer[2];
    tmr[3]->clk = clk->clk_timer[3];
    tmr[4]->clk = clk->clk_timer[4];
    tmr[5]->clk = clk->clk_timer[5];

    /* SPIs */
    spi[0] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    spi[1] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    spi[2] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    memory_region_add_subregion(mem, 0xB0060000, &spi[0]->iomem);
    memory_region_add_subregion(mem, 0xB0061000, &spi[1]->iomem);
    memory_region_add_subregion(mem, 0xB0062000, &spi[2]->iomem);

    /* UARTs */
    ser[0] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[1] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[2] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[3] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[4] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[5] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[6] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[7] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[8] = NUC980_SER(object_new(TYPE_NUC980_SER));
    ser[9] = NUC980_SER(object_new(TYPE_NUC980_SER));
    nuc980_ser_chardev_attach(ser[0], serial_hd(0));
    nuc980_ser_chardev_attach(ser[1], serial_hd(1));
    nuc980_ser_chardev_attach(ser[2], serial_hd(2));
    nuc980_ser_chardev_attach(ser[3], serial_hd(3));
    nuc980_ser_chardev_attach(ser[4], serial_hd(4));
    nuc980_ser_chardev_attach(ser[5], serial_hd(5));
    nuc980_ser_chardev_attach(ser[6], serial_hd(6));
    nuc980_ser_chardev_attach(ser[7], serial_hd(7));
    nuc980_ser_chardev_attach(ser[8], serial_hd(8));
    nuc980_ser_chardev_attach(ser[9], serial_hd(9));
    memory_region_add_subregion(mem, 0xB0070000, &ser[0]->iomem);
    memory_region_add_subregion(mem, 0xB0071000, &ser[1]->iomem);
    memory_region_add_subregion(mem, 0xB0072000, &ser[2]->iomem);
    memory_region_add_subregion(mem, 0xB0073000, &ser[3]->iomem);
    memory_region_add_subregion(mem, 0xB0074000, &ser[4]->iomem);
    memory_region_add_subregion(mem, 0xB0075000, &ser[5]->iomem);
    memory_region_add_subregion(mem, 0xB0076000, &ser[6]->iomem);
    memory_region_add_subregion(mem, 0xB0077000, &ser[7]->iomem);
    memory_region_add_subregion(mem, 0xB0078000, &ser[8]->iomem);
    memory_region_add_subregion(mem, 0xB0079000, &ser[9]->iomem);
    ser[0]->irq = irq[36];

    /* PDMA */
    dma = NUC980_DMA(object_new(TYPE_NUC980_DMA));
    memory_region_add_subregion(mem, 0xB0080000, &dma->iomem);
    dma->irq[0] = irq[25];
    dma->irq[1] = irq[26];

    /* load boot flash from file */
    nuc980_soc_ldflash(machine);
}

static void nuc980_soc_class_init(ObjectClass *obj_class, void *data)
{
    MachineClass *machine_class = MACHINE_CLASS(obj_class);

    machine_class->desc     = "Nuvoton NUC980 SoC (ARM926EJ-S)";
    machine_class->reset    = nuc980_soc_reset;
    machine_class->init     = nuc980_soc_instance_init;
}

/*------------------------------ SOC TYPE -----------------------------------*/

static const TypeInfo nuc980_soc_type = {
    .name          = TYPE_NUC980_SOC,
    .parent        = TYPE_MACHINE,
    .class_init    = nuc980_soc_class_init,
};

/*****************************************************************************/
/*                                REGISTERATION                              */
/*****************************************************************************/

static void nuc980_register_types(void)
{
    type_register_static(&nuc980_sys_type);
    type_register_static(&nuc980_clk_type);
    type_register_static(&nuc980_sdr_type);
    type_register_static(&nuc980_pic_type);
    type_register_static(&nuc980_gpi_type);
    type_register_static(&nuc980_eth_type);
    type_register_static(&nuc980_usb_type);
    type_register_static(&nuc980_fmi_type);
    type_register_static(&nuc980_rtc_type);
    type_register_static(&nuc980_tmr_type);
    type_register_static(&nuc980_spi_type);
    type_register_static(&nuc980_ser_type);
    type_register_static(&nuc980_dma_type);
    type_register_static(&nuc980_soc_type);
}

type_init(nuc980_register_types)

