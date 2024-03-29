#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <stdint.h>
#include <libdragon.h>
#include <stdlib.h>
#include "crc32.h"
#include "rtc.h"
#include "ini.h"

#define MAJOR_VERSION (0x00)
#define MINOR_VERSION (0x02)
#define DEBUG_VERSION (0x00)
#define MENU_VERSION ((MAJOR_VERSION << 16) | (MINOR_VERSION << 8) | DEBUG_VERSION)

typedef volatile uint32_t vu32;
typedef volatile uint64_t vu64;

//boot simulation
#define PI_STATUS_REG *(vu32*)(0xA4600010)
#define PI_BSD_DOM1_LAT_REG *(vu32*)(0xA4600014)
#define PI_BSD_DOM1_PWD_REG *(vu32*)(0xA4600018)
#define PI_BSD_DOM1_PGS_REG *(vu32*)(0xA460001C)
#define PI_BSD_DOM1_RLS_REG *(vu32*)(0xA4600020)

#define SP_STATUS_REG           *(vu32*)(0xA4040010)
#define SP_DMA_BUSY_REG         *(vu32*)(0xA4040018)
#define VI_V_INTR_REG           *(vu32*)(0xA440000C)
#define VI_H_VIDEO_REG          *(vu32*)(0xA4400024)
#define VI_V_CURRENT_LINE_REG   *(vu32*)(0xA4400010)
#define AI_DRAM_ADDR_REG        *(vu32*)(0xA4500000)
#define AI_LEN_REG              *(vu32*)(0xA4500004)
#define DPC_STATUS_REG          *(vu32*)(0xA410000C)

/** @brief Memory location to read which determines the TV type. */
#define TV_TYPE_LOC         0x80000300

#define CART_CONTROL_REG        (0x18000000)
#define CART_VERSION_REG        (0x18000004)
#define CART_SAVE_OFFSET_REG    (0x1800000C)
#define CART_BACKUP_REG         (0x18000010)
#define CART_MAPPING0_REG       (0x18000080)

enum cictype
{
    E_CICTYPE_UNKNOWN,
    E_CICTYPE_6101,
    E_CICTYPE_6102,
    E_CICTYPE_6103,
    E_CICTYPE_6105,
    E_CICTYPE_6106,
};

enum savetype
{
    E_SAVETYPE_UNKNOWN,
    E_SAVETYPE_NONE,
    E_SAVETYPE_EEP4K,
    E_SAVETYPE_EEP16K,
    E_SAVETYPE_SRAM32,
    E_SAVETYPE_SRAM32x3,
    E_SAVETYPE_FLASH
};

#define TV_PAL 0
#define TV_NTSC 1

struct rom_config_st
{
    char id[3];
    char name[33];
    int8_t tv;
    uint8_t cic;
    uint8_t save;
    uint32_t save_offset;
    uint32_t rom_size;
    uint32_t rom_crc;
    uint8_t mapping_regs[32];

};

struct cart_config_st
{
    uint8_t num_roms;
    int8_t autoboot_index;
    struct rom_config_st * rom_configs;
};

struct CartVersion_st {
    uint8_t Id;
    uint8_t Major;
    uint8_t Minor;
    uint8_t Debug;
} __attribute__ ((packed));

union CartVersion_un
{
    uint32_t val;
    struct CartVersion_st str;
};

struct menu_st
{
    display_context_t disp;
    
    uint8_t cursor_pos;
    int cursor_anim;
    bool cursor_down ;
    bool cursor_up;
    bool cursor_right;
    bool cursor_left;
    bool last_down;
    bool last_up;
    bool last_right;
    bool last_left;
    
    struct cart_config_st cart_config;
    int8_t selected_index;
    struct rom_config_st selected_config;
    bool is_crc_checked;
    uint32_t rom_crc;
    
    struct controller_data keysDown;
    struct controller_data keysPressed;

    struct CartVersion_st CartVersion;
    
};

struct BackupRegister_st {
    unsigned IsValid : 1;
    unsigned TvType : 1;
    unsigned CicType : 3;
    unsigned SaveType : 3;
    unsigned SelectedIndex : 8;
    unsigned SaveOffset : 10;
} __attribute__ ((packed));

union BackupRegister_un
{
    uint32_t val;
    struct BackupRegister_st str;
};

// Array with compatible cart versions
const struct CartVersion_st _SupportedCartVersions[] = {
    {.Id = 4, .Major = 0, .Minor = 3, .Debug = 0},
    {.Id = 5, .Major = 0, .Minor = 1, .Debug = 0}
};


const int _CursorAnimationAlpha[] = { 10, 10, 11, 13, 16, 19, 23, 28, 
    33, 38, 44, 50, 56, 63, 69, 75, 81, 86, 91, 96, 100, 103, 106, 108, 
    109, 110, 109, 108, 106, 103, 100, 96, 91, 86, 81, 75, 69, 63, 56, 
    50, 44, 38, 33, 28, 23, 19, 16, 13, 11, 10, 10 };

struct CartHeader_st {
    uint32_t InitialPiLat;
    uint32_t ClockRate;
    uint32_t EntryPoint;
    uint32_t Release;
    uint32_t Crc1;
    uint32_t Crc2;
    char __RESERVED1[8];
    char ImageName[20];
    char __RESERVED2[7];
    char CartridgeId[5];
} __attribute__ ((packed));

void PrepareReset(void)
{
    while ((SP_STATUS_REG & 0x01) == 0);
    MEMORY_BARRIER();
    
    SP_STATUS_REG = 0x000A;
    MEMORY_BARRIER();
    
    while (SP_DMA_BUSY_REG & 0x01);
    MEMORY_BARRIER();
    
    PI_STATUS_REG = 3;
    VI_V_INTR_REG = 0x03FF;
    VI_H_VIDEO_REG = 0;
    VI_V_CURRENT_LINE_REG = 0;
    AI_DRAM_ADDR_REG = 0;
    AI_LEN_REG = 0;
    
    MEMORY_BARRIER();
    
    while (SP_STATUS_REG & 0x04) ;
    MEMORY_BARRIER();
    
    if (DPC_STATUS_REG & 0x01)
    {
        while (DPC_STATUS_REG & 0x20) ;
    }
}

// THX to saturnu for providing this tutorial:
// https://circuit-board.de/forum/index.php/Thread/8939-N64-und-ED64-Programme-erstellen/

const uint16_t _CicSeeds[] = {
	0x3f3f, 0x3f3f, 0x3f3f, 0x7878, 0x9191, 0x8585, 0xDDDD, 0
};

// Simulated PIF ROM bootcode adapted from DaedalusX64 emulator
void simulate_pif_boot(uint32_t cic_chip, uint8_t tv)
{
    unsigned int gBootCic = E_CICTYPE_6102; //added

    uint32_t ix, sz;
    vu32 *dst;
    vu64 *gGPR = (vu64 *) 0xA0080000;
    
    //char country = tv ? 'P' : 'E';

// clear XBUS/Flush/Freeze
    ((vu32 *) 0xA4100000)[3] = 0x15;

// copy the memsize for different boot loaders
    sz = (gBootCic != E_CICTYPE_6105) ? *(vu32 *) 0xA0000318 : *(vu32 *) 0xA00003F0;
    if (cic_chip == E_CICTYPE_6105)
        *(vu32 *) 0xA00003F0 = sz;
    else
        *(vu32 *) 0xA0000318 = sz;

// clear some OS globals for cleaner boot
    *(vu32 *) 0xA000030C = 0; // cold boot
    memset((void *) 0xA000031C, 0, 64); // clear app nmi buffer

// clear memory outside boot segment
//memset((void *)0xA0100000, 0, sz - 0x00100000); //hack enable

    // clear SP_DMEM and SP_IMEM
    dst = (vu32 *)0xA4000000;
    for (ix = 0x40/4; ix < 0x2000/4; ix++)
        dst[ix] = 0;
    
// Copy low 0x1000 bytes to DMEM
    dst = (vu32 *) 0xA4000000;
    for (ix = 0; ix < (0x1000 >> 2); ix++)
        dst[ix] = io_read(0xB0000000 + (ix << 2));

// Need to copy crap to IMEM for CIC-6105 boot.
    dst = (vu32 *) 0xA4001000;

// register values due to pif boot for CiC chip and country code, and IMEM crap
    gGPR[1] = 0;
    gGPR[2] = 0;
    gGPR[3] = 0;
    gGPR[4] = 0;
    gGPR[5] = 0;
    gGPR[6] = 0;
    gGPR[7] = 0;
    gGPR[8] = 0;
    gGPR[9] = 0;
    gGPR[10] = 0;
    gGPR[11] = 0; // t3
    gGPR[12] = 0;
    gGPR[13] = 0;
    gGPR[14] = 0;
    gGPR[15] = 0;
    gGPR[16] = 0; // s0
    gGPR[17] = 0; // s1
    gGPR[18] = 0; // s2
    gGPR[19] = 0; // s3
    gGPR[20] = 0; // s4
    gGPR[21] = 0; // s5 0: Cold Reset, 1: Reset Button
    gGPR[22] = 0; // s6
    gGPR[23] = 0; // s7
    gGPR[24] = 0;
    gGPR[25] = 0;
    gGPR[26] = 0;
    gGPR[27] = 0;
    gGPR[28] = 0;
    gGPR[29] = 0; // sp
    gGPR[30] = 0;
    gGPR[31] = 0; // ra
    
    gGPR[11] = 0xFFFFFFFFA4000040LL; // t3
    gGPR[22] = _CicSeeds[cic_chip] >> 8; // s6
    gGPR[29] = 0xFFFFFFFFA4001FF0LL; // sp
    if (tv == 0) // PAL
    {
        gGPR[23] = 6; // s7
        gGPR[31] = 0xFFFFFFFFA4001554LL; // ra
        dst[0x04 >> 2] = 0xBDA807FC;
    }
    else
    {
        gGPR[20] = 1; // s4
        gGPR[31] = 0xFFFFFFFFA4001550LL; // ra
        dst[0x04 >> 2] = 0x8DA807FC;
    }
    
    dst[0x00 >> 2] = 0x3C0DBFC0;
    dst[0x08 >> 2] = 0x25AD07C0;
    dst[0x0C >> 2] = 0x31080080;
    dst[0x10 >> 2] = 0x5500FFFC;
    dst[0x14 >> 2] = 0x3C0DBFC0;
    dst[0x18 >> 2] = 0x8DA80024;
    dst[0x1C >> 2] = 0x3C0BB000;
    
    // read ROM ID from Cart Header (was copied to DMEM before)
    uint16_t romId = (uint16_t)(*(vu32 *)0xA400003C >> 16);
    
    switch (romId)
    {
    case 0x4258: // 'BX' => BattleTanx
    case 0x4251: // 'BQ' => BattleTanx GA
		// BattleTanx will not boot otherwise
        PI_BSD_DOM1_LAT_REG = 0x80;
        PI_BSD_DOM1_RLS_REG = 0x37;
        PI_BSD_DOM1_PWD_REG = 0x12;
        PI_BSD_DOM1_PGS_REG = 0x40;
            break;
    }
    
    // set HW registers
    PI_STATUS_REG = 3;

    // now set MIPS registers - set CP0, and then GPRs, then jump thru gpr11 (which is 0xA400040)
    asm(".set noat\n\t"
        ".set noreorder\n\t"
        "li $8,0x34000000\n\t"
        "mtc0 $8,$12\n\t"
        "nop\n\t"
        "li $9,0x0006E463\n\t"
        "mtc0 $9,$16\n\t"
        "nop\n\t"
        "li $8,0x00005000\n\t"
        "mtc0 $8,$9\n\t"
        "nop\n\t"
        "li $9,0x0000005C\n\t"
        "mtc0 $9,$13\n\t"
        "nop\n\t"
        "li $8,0x007FFFF0\n\t"
        "mtc0 $8,$4\n\t"
        "nop\n\t"
        "li $9,0xFFFFFFFF\n\t"
        "mtc0 $9,$14\n\t"
        "nop\n\t"
        "mtc0 $9,$30\n\t"
        "nop\n\t"
        "lui $8,0\n\t"
        "mthi $8\n\t"
        "nop\n\t"
        "mtlo $8\n\t"
        "nop\n\t"
        "ctc1 $8,$31\n\t"
        "nop\n\t"
        "lui $31,0xA008\n\t"
        "ld $1,0x08($31)\n\t"
        "ld $2,0x10($31)\n\t"
        "ld $3,0x18($31)\n\t"
        "ld $4,0x20($31)\n\t"
        "ld $5,0x28($31)\n\t"
        "ld $6,0x30($31)\n\t"
        "ld $7,0x38($31)\n\t"
        "ld $8,0x40($31)\n\t"
        "ld $9,0x48($31)\n\t"
        "ld $10,0x50($31)\n\t"
        "ld $11,0x58($31)\n\t"
        "ld $12,0x60($31)\n\t"
        "ld $13,0x68($31)\n\t"
        "ld $14,0x70($31)\n\t"
        "ld $15,0x78($31)\n\t"
        "ld $16,0x80($31)\n\t"
        "ld $17,0x88($31)\n\t"
        "ld $18,0x90($31)\n\t"
        "ld $19,0x98($31)\n\t"
        "ld $20,0xA0($31)\n\t"
        "ld $21,0xA8($31)\n\t"
        "ld $22,0xB0($31)\n\t"
        "ld $23,0xB8($31)\n\t"
        "ld $24,0xC0($31)\n\t"
        "ld $25,0xC8($31)\n\t"
        "ld $26,0xD0($31)\n\t"
        "ld $27,0xD8($31)\n\t"
        "ld $28,0xE0($31)\n\t"
        "ld $29,0xE8($31)\n\t"
        "ld $30,0xF0($31)\n\t"
        "ld $31,0xF8($31)\n\t"
        "jr $11\n\t"
        "nop"
        ::: "$8");
}

uint8_t get_cic(uint32_t crc)
{
    uint8_t res;
    
    switch (crc)
    {
    case 0x6170A4A1:    // Starfox 64 (6101)
    case 0x009E9EA3:    // Lylat Wars (7102)
        res = E_CICTYPE_6101;
        break;
    case 0x90BB6CB5:
        res = E_CICTYPE_6102;
        break;
    case 0x0B050EE0:
        res = E_CICTYPE_6103;
        break;
    case 0x98BC2C86:
        res = E_CICTYPE_6105;
        break;
    case 0xACC8580A:
        res = E_CICTYPE_6106;
        break;
    default:
        res = E_CICTYPE_UNKNOWN;
        break;
    }
    
    return res;
}

const char * get_cic_str(int cic)
{
    const char * res;
    
    switch (cic)
    {
    case E_CICTYPE_6101:
        res = "6101";
        break;
    case E_CICTYPE_6102:
        res = "6102";
        break;
    case E_CICTYPE_6103:
        res = "6103";
        break;
    case E_CICTYPE_6105:
        res = "6105";
        break;
    case E_CICTYPE_6106:
        res = "6106";
        break;
    default:
        res = "UNKNOWN";
        break;
    }
    
    return res;
}

enum cictype get_cic_from_str(const char * str)
{
    if (str == NULL)
        return E_CICTYPE_UNKNOWN;
    
    if (strcmp(str, "6101") == 0)
        return E_CICTYPE_6101;
    
    if (strcmp(str, "6102") == 0)
        return E_CICTYPE_6102;
    
    if (strcmp(str, "6103") == 0)
        return E_CICTYPE_6103;
    
    if (strcmp(str, "6105") == 0)
        return E_CICTYPE_6105;
    
    if (strcmp(str, "6106") == 0)
        return E_CICTYPE_6106;
    
    return E_CICTYPE_UNKNOWN;
}

int8_t get_tv_type_by_id(char id)
{
    int res;
    
    switch (id)
    {
    case 'E': // USA
    case 'J': // Japan
    case 'U': // Australia
    case 'A': // X_NTSC
    case '7': // NTSC_BETA
        res = TV_NTSC;
        break;
        
    case 'P': // Europe
    case 'D': // Germany
    case 'F': // France
    case 'I': // Italy
    case 'S': // Spain
    case 'X': // X_PAL
    case 'Y': // Y_PAL
        res = TV_PAL;
        break;
        
    default:
        res = -1;
    }
    
    return res;
}

const char * get_tvtype_str(int tv)
{
    const char * res;
    
    switch (tv)
    {
    case TV_PAL:
        res = "PAL";
        break;
    case TV_NTSC:
        res = "NTSC";
        break;
    default:
        res = "UNKNOWN";
        break;
    }
    
    return res;
}

const char * get_savetype_str(enum savetype save)
{
    const char * res;
    
    switch (save)
    {
    case E_SAVETYPE_NONE:
        res = "OFF";
        break;
    case E_SAVETYPE_EEP4K:
        res = "EEP4K";
        break;
    case E_SAVETYPE_EEP16K:
        res = "EEP16K";
        break;
    case E_SAVETYPE_SRAM32:
        res = "SRAM32";
        break;
    case E_SAVETYPE_SRAM32x3:
        res = "SRAM32x3";
        break;
    case E_SAVETYPE_FLASH:
        res = "FLASHRAM";
        break;
    
    case E_SAVETYPE_UNKNOWN:
    default:
        res = "UNKNOWN";
        break;
    }
    
    return res;
}

enum savetype get_savetype_from_str(const char * str)
{
    if (str == NULL)
        return E_SAVETYPE_UNKNOWN;
    
    if (strcmp(str, "OFF") == 0)
        return E_SAVETYPE_NONE;
    
    if (strcmp(str, "EEP4K") == 0)
        return E_SAVETYPE_EEP4K;
    
    if (strcmp(str, "EEP16K") == 0)
        return E_SAVETYPE_EEP16K;
    
    if (strcmp(str, "SRAM32") == 0)
        return E_SAVETYPE_SRAM32;
    
    if (strcmp(str, "SRAM32x3") == 0)
        return E_SAVETYPE_SRAM32x3;
    
    if (strcmp(str, "FLASHRAM") == 0)
        return E_SAVETYPE_FLASH;
        
    return E_SAVETYPE_UNKNOWN;
}

const char * get_cart_hw_str(uint8_t id)
{
    const char * res;

    switch (id)
    {
        case 4:
            res = "1.0";
            break;
        case 5:
            res = "2.0";
            break;
        default:
            res = "???";
    }
    return res;
}

void cart_set_savetype(enum savetype save)
{
    uint32_t reg;
    
    reg = io_read(CART_CONTROL_REG);
    
    reg &= ~0x1E;
    
    switch (save)
    {
        case E_SAVETYPE_EEP4K:
            reg |= 0x02;
            break;
        case E_SAVETYPE_EEP16K:
            reg |= 0x06;
            break;
        case E_SAVETYPE_SRAM32:
        case E_SAVETYPE_SRAM32x3:
            reg |= 0x08;
            break;
        case E_SAVETYPE_FLASH:
            reg |= 0x10;
            break;
            
        case E_SAVETYPE_NONE:
        case E_SAVETYPE_UNKNOWN:
        default:
            break;
    }
    
    io_write(CART_CONTROL_REG, reg);
}

void cart_enable_gamerom(bool is_enabled)
{
    uint32_t reg;
    
    reg = io_read(CART_CONTROL_REG);
    
    reg &= ~0x01;
    
    if (is_enabled)
    {
        reg |= 1;
    }
    
    io_write(CART_CONTROL_REG, reg);
}

void cart_enable_mapping(bool is_enabled)
{
    uint32_t reg;
    
    reg = io_read(CART_CONTROL_REG);
    
    reg &= ~0x20;
    
    if (is_enabled)
    {
        reg |= 0x20;
    }
    
    io_write(CART_CONTROL_REG, reg);
}

void cart_set_mapping(uint8_t * regs)
{
    if (regs)
    {
        for (int i = 0; i < 32; i++)
            io_write(CART_MAPPING0_REG + i * sizeof(uint32_t), regs[i]);
    }
}

void cart_set_save_offset(uint32_t offset)
{
    io_write(CART_SAVE_OFFSET_REG, offset);
}

struct CartVersion_st cart_get_version(void)
{
    union CartVersion_un cartVersion;
    cartVersion.val = io_read(CART_VERSION_REG);
    return cartVersion.str;
}

struct BackupRegister_st cart_get_backup(void)
{
    union BackupRegister_un bak;

    bak.val = io_read(CART_BACKUP_REG);
    return bak.str;
}

void cart_set_backup(uint32_t value)
{
    io_write(CART_BACKUP_REG, value);
}

// check if there is a version with the found ID and Major version in the array
// the minor version must be greater or equal to the entry in the array
bool check_cart_version(const struct CartVersion_st * cartVersion)
{
    bool isCompatible = false;
    for (int i = 0; i < sizeof(_SupportedCartVersions)/sizeof(struct CartVersion_st); i++)
    {
        const struct CartVersion_st * cv = &_SupportedCartVersions[i];
        if ((cartVersion->Id == cv->Id)
            && ((cartVersion->Major == cv->Major))
            && (cartVersion->Minor >= cv->Minor))
        {
            isCompatible = true;
            break;
        }
    }
    return isCompatible;
}

// no return from here
void show_cart_incompatible(struct menu_st *menu)
{
    char sStr[64];

    display_init( RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE );
    timer_init();

    while(1)
    {
        
        /* Grab a render buffer */
        while( !(menu->disp = display_lock()) );
        
        controller_scan();

        /*Fill the screen */
        graphics_fill_screen( menu->disp, 0 );

        /* Set the text output color */
        graphics_set_color( 0xFFFFFFFF, 0 );
        
        sprintf(sStr, "Build = %s %s", __DATE__, __TIME__);
        graphics_draw_text( menu->disp, 20, 15, sStr);
        sprintf(sStr, "Menu v%d.%d.%d / FPGA [%s] v%d.%d.%d",
            MAJOR_VERSION, MINOR_VERSION, DEBUG_VERSION,
            get_cart_hw_str(menu->CartVersion.Id), menu->CartVersion.Major, menu->CartVersion.Minor, menu->CartVersion.Debug);
        graphics_draw_text( menu->disp, 20, 25, sStr);
        graphics_draw_text( menu->disp, 70, 40, "==== Brutzelkarte ====" );
        
        graphics_set_color( 0xFF1010FF, 0 );
        graphics_draw_text( menu->disp, 40, 90, "The menu is not compatible");
        graphics_draw_text( menu->disp, 40, 100, "compatible with the FPGA.");

        graphics_set_color( 0xFFFF00FF, 0 );
        graphics_draw_text( menu->disp, 20, 130, "Please update the FPGA or the menu!");

        display_show(menu->disp);
    }
}

void menu_process_keys(struct menu_st *menu)
{
    menu->cursor_down = false;
    menu->cursor_up = false;
    menu->cursor_right = false;
    menu->cursor_left = false;
    
    if (menu->keysPressed.c[0].y < -30)
    {
        if (menu->last_down == false)
        {
            menu->cursor_down = true;
        }
        menu->last_down = true;
    }
    if (menu->keysPressed.c[0].y > -20)
    {
        menu->last_down = false;
    }
    if (menu->keysDown.c[0].down)
    {
        menu->cursor_down = true;
    }
    
    if (menu->keysPressed.c[0].y > 30)
    {
        if (menu->last_up == false)
        {
            menu->cursor_up = true;
        }
        menu->last_up = true;
    }
    if (menu->keysPressed.c[0].y < 20)
    {
        menu->last_up = false;
    }
    if (menu->keysDown.c[0].up)
    {
        menu->cursor_up = true;
    }
    
    if (menu->keysPressed.c[0].x < -30)
    {
        if (menu->last_left == false)
        {
            menu->cursor_left = true;
        }
        menu->last_left = true;
    }
    if (menu->keysPressed.c[0].x > -20)
    {
        menu->last_left = false;
    }
    if (menu->keysDown.c[0].left)
    {
        menu->cursor_left = true;
    }
    
    if (menu->keysPressed.c[0].x > 30)
    {
        if (menu->last_right == false)
        {
            menu->cursor_right = true;
        }
        menu->last_right = true;
    }
    if (menu->keysPressed.c[0].x < 20)
    {
        menu->last_right = false;
    }
    if (menu->keysDown.c[0].right)
    {
        menu->cursor_right = true;
    }
}

void start_game(int8_t tv, uint8_t cic)
{
    *(uint32_t *) TV_TYPE_LOC = tv;
    disable_interrupts();
    PrepareReset();
    simulate_pif_boot(cic, tv);
}

void save_backup_data(struct menu_st *menu)
{
    struct BackupRegister_st backup = { 0 };
    backup.IsValid = 1;
    backup.TvType = menu->selected_config.tv;
    backup.CicType = menu->selected_config.cic;
    backup.SaveType = menu->selected_config.save;
    backup.SelectedIndex = menu->selected_index;
    backup.SaveOffset = menu->selected_config.save_offset;

    void * tmp = &backup;
    cart_set_backup(*(uint32_t *)tmp);
}

void load_config(struct menu_st * menu)
{
    int dfsStatus = dfs_init(0xB0200000);
    if(dfsStatus == DFS_ESUCCESS)
    {
        ini_t *ini = ini_load("rom://brutzelkarte/config.ini");
        if (ini)
        {
            bool has_error = false;
            // this loop is just to have a breakpoint
            do
            {
                const char *tmp_str = ini_get(ini, "CART", "NUM_ROMS");
                if (tmp_str == NULL)
                    break;
                uint8_t num_roms = atoi(tmp_str);
                
                tmp_str = ini_get(ini, "CART", "AUTOBOOT");
                if (tmp_str == NULL)
                    break;
                int8_t autoboot_index = atoi(tmp_str);
                
                menu->cart_config.rom_configs = (struct rom_config_st *)calloc(num_roms, sizeof(struct rom_config_st));
                if (menu->cart_config.rom_configs == NULL)
                    break;
                
                for (int i = 0; i < num_roms; i++)
                {
                    menu->cart_config.rom_configs[i].tv = -1;
                    struct rom_config_st tmp_cfg;
                    
                    char rom_index[10];
                    sprintf(rom_index, "ROM%d", i);
                    
                    tmp_str = ini_get(ini, rom_index, "ID");
                    
                    // allow ID to be empty
                    if (tmp_str == NULL)
                    {
                        memset(tmp_cfg.id, 0, sizeof(tmp_cfg.id));
                    }
                    else
                    {
                        strncpy(tmp_cfg.id, tmp_str, sizeof(tmp_cfg.id) - 1);
                    }
                    
                    tmp_str = ini_get(ini, rom_index, "NAME");
                    
                    // allow NAME to be empty
                    if (tmp_str == NULL)
                    {
                        memset(tmp_cfg.name, 0, sizeof(tmp_cfg.name));
                    }
                    else
                    {
                        strncpy(tmp_cfg.name, tmp_str, sizeof(tmp_cfg.name) - 1);
                    }
                    
                    tmp_str = ini_get(ini, rom_index, "TV");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    if (strcmp(tmp_str, "PAL") == 0)
                        tmp_cfg.tv = TV_PAL;
                    else
                        tmp_cfg.tv = TV_NTSC;
                    
                    tmp_str = ini_get(ini, rom_index, "CIC");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    tmp_cfg.cic = get_cic_from_str(tmp_str);
                    
                    tmp_str = ini_get(ini, rom_index, "SAVE");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    tmp_cfg.save = get_savetype_from_str(tmp_str);
                    
                    tmp_str = ini_get(ini, rom_index, "SAVE_OFFSET");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    tmp_cfg.save_offset = atoi(tmp_str);
                    
                    tmp_str = ini_get(ini, rom_index, "ROM_SIZE");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    tmp_cfg.rom_size = atoi(tmp_str);
                    
                    tmp_str = ini_get(ini, rom_index, "ROM_CRC");
                    if (tmp_str == NULL)
                    {
                        has_error = true;
                        break;
                    }
                    tmp_cfg.rom_crc = strtoul(tmp_str, NULL, 16);
                    
                    // read the mapping registers
                    for (int map = 0; map < 32; map++)
                    {
                        char map_index[12];
                        sprintf(map_index, "MAPPING%d", map);
                        
                        tmp_str = ini_get(ini, rom_index, map_index);
                        if (tmp_str == NULL)
                        {
                            has_error = true;
                            break;
                        }
                        tmp_cfg.mapping_regs[map] = atoi(tmp_str);
                    }
                    
                    // error in loop?
                    if (has_error)
                    {
                        break;
                    }
                    
                    // Config for this rom complete - add to menu
                    menu->cart_config.rom_configs[i] = tmp_cfg;
                    menu->cart_config.num_roms++;
                }
                
                if (has_error)
                    break;
                
                menu->cart_config.autoboot_index = autoboot_index;
            }
            while (0);
            
            ini_free(ini);
        }
    }
}

int main(void)
{
    struct menu_st menu;
    struct CartHeader_st * cartInfo;
    
    uint32_t cartData[4096/4];
    uint32_t bootCodeCrc = 0;
    
    char sStr[64];
    
    /* enable interrupts (on the CPU) */
    init_interrupts();
    
    int64_t tick = 0;
    uint8_t blink = 0;
    
    memset(cartData, 0, sizeof(cartData));
    memset(&menu, 0, sizeof(struct menu_st));
    menu.cart_config.autoboot_index = -1;
    
    CRC_BuildTable();
    
    controller_init();
    
    // read version
    menu.CartVersion = cart_get_version();

    // check compatibility
    if (!check_cart_version(&menu.CartVersion))
        show_cart_incompatible(&menu);
    
    // check the backup register to see if it has valid data
    struct BackupRegister_st cartBackup = cart_get_backup();
    if (cartBackup.IsValid)
    {
        menu.selected_index = cartBackup.SelectedIndex;
        
        // autostart the game
        // (can be interrupted by holding Z)
        controller_scan();
        menu.keysPressed = get_keys_pressed();
        if ((menu.keysPressed.c[0].err == ERROR_NONE)
            && (!(menu.keysPressed.c[0].Z)))
        {
            cart_set_save_offset(cartBackup.SaveOffset);
            cart_set_savetype(cartBackup.SaveType);
            cart_enable_gamerom(true);
            cart_enable_mapping(true);
            start_game(cartBackup.TvType, cartBackup.CicType);
            
            // should not get here
            while (1) ;
        }
        else
        {
            // invalidate backup data
            cart_set_backup(0);
        }
    }
    
    load_config(&menu);
    
    if ((menu.cart_config.autoboot_index >= 0) && (menu.cart_config.num_roms > menu.cart_config.autoboot_index))
    {
        menu.selected_index = menu.cart_config.autoboot_index;
    }
    
    if (menu.cart_config.num_roms > menu.selected_index)
    {
        menu.selected_config = menu.cart_config.rom_configs[menu.selected_index];
    }
    else
    {
        menu.selected_index = 0;
        if (menu.cart_config.num_roms == 0)
        {
            // create dummy config
            menu.cart_config.rom_configs = (struct rom_config_st *)calloc(1, sizeof(struct rom_config_st));
            menu.cart_config.num_roms = 1;
        }
    }
    
    // switch to game
    cart_enable_gamerom(true);
    cart_enable_mapping(true);
    cartInfo = (struct CartHeader_st *)cartData;
    
    if (menu.cart_config.autoboot_index == menu.selected_index)
    {
        // check if the autoboot config matches the rom id
        cart_set_mapping(menu.selected_config.mapping_regs);
        cart_set_save_offset(menu.selected_config.save_offset);
        data_cache_hit_writeback_invalidate(cartData, sizeof(cartData));
        dma_read(cartData, 0, sizeof(cartData));
        data_cache_hit_invalidate(cartData, sizeof(cartData));
        bootCodeCrc = CRC_Calculate(&cartData[16], 4032);
        
        // check if the autoboot config matches the rom id
        if (memcmp(&cartInfo->CartridgeId[1], menu.cart_config.rom_configs[menu.cart_config.autoboot_index].id, 2) == 0)
        {
            // autostart the game
            // (can be interrupted by holding Z)
            controller_scan();
            menu.keysPressed = get_keys_pressed();
            if ((menu.keysPressed.c[0].err == ERROR_NONE)
                && (!(menu.keysPressed.c[0].Z)))
            {
                save_backup_data(&menu);
                cart_set_savetype(menu.selected_config.save);
                start_game(menu.selected_config.tv, menu.selected_config.cic);
                
                // should not get here
                while (1) ;
            }
        }
        else
        {
            menu.cart_config.autoboot_index = -1;
            menu.selected_index = 0;
        }
    }
    menu.selected_config = menu.cart_config.rom_configs[menu.selected_index];
    
    // autoboot not executed -> show menu
    cart_set_mapping(menu.selected_config.mapping_regs);
    cart_set_save_offset(menu.selected_config.save_offset);
    data_cache_hit_writeback_invalidate(cartData, sizeof(cartData));
    dma_read(cartData, 0, sizeof(cartData));
    data_cache_hit_invalidate(cartData, sizeof(cartData));
    bootCodeCrc = CRC_Calculate(&cartData[16], 4032);
    
    /* Initialize peripherals */
    //*(uint32_t *) TV_TYPE_LOC = TV_NTSC;
    display_init( RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE );
    timer_init();
    
    /* Main loop test */
    while(1)
    {
        
        /* Grab a render buffer */
        while( !(menu.disp = display_lock()) );
        
        controller_scan();

        menu.keysDown = get_keys_down();
        menu.keysPressed = get_keys_pressed();
        
        // clear input data in case of an error
        if (menu.keysDown.c[0].err != ERROR_NONE)
        {
            menu.keysDown.c[0].data = 0;
            menu.keysPressed.c[0].data = 0;
        }
        
        menu_process_keys(&menu);
        
        if (menu.cursor_down)
        {
            if (menu.cursor_pos < 2)
                menu.cursor_pos++;
        }
        
        if (menu.cursor_up)
        {
            if (menu.cursor_pos > 0)
                menu.cursor_pos--;
        }
        
        switch (menu.cursor_pos)
        {
        case 0:
            if (menu.cursor_right)
            {
                if (menu.selected_config.tv < 1)
                    menu.selected_config.tv++;
            }
            if (menu.cursor_left)
            {
                if (menu.selected_config.tv > 0)
                    menu.selected_config.tv--;
            }
            break;
            
        case 1:
            if (menu.cursor_right)
            {
                if (menu.selected_config.cic < 5)
                    menu.selected_config.cic++;
            }
            if (menu.cursor_left)
            {
                if (menu.selected_config.cic > 1)
                    menu.selected_config.cic--;
            }
            break;
            
        case 2:
            if (menu.cursor_right)
            {
                if (menu.selected_config.save < 6)
                    menu.selected_config.save++;
            }
            if (menu.cursor_left)
            {
                if (menu.selected_config.save > 1)
                    menu.selected_config.save--;
            }
            break;
        }
        
        bool is_rom_changed = false;
        if (menu.keysDown.c[0].C_right)
        {
            if (menu.selected_index < menu.cart_config.num_roms - 1)
            {
                menu.selected_index++;
                is_rom_changed = true;
            }
        }
        else if (menu.keysDown.c[0].C_left)
        {
            if (menu.selected_index > 0)
            {
                menu.selected_index--;
                is_rom_changed = true;
            }
        }
        
        if (menu.keysDown.c[0].L)
        {
			uint8_t crc_buffer[4096];
			CRC_Initialize(&menu.rom_crc);
			uint32_t read_pos = 0;
			
			while (read_pos < menu.selected_config.rom_size)
			{
				uint32_t part = sizeof(crc_buffer);
				if (part > menu.selected_config.rom_size - read_pos)
					part = menu.selected_config.rom_size - read_pos;
				data_cache_hit_writeback_invalidate(crc_buffer, sizeof(crc_buffer));
				dma_read(crc_buffer, read_pos, part);
				data_cache_hit_invalidate(crc_buffer, sizeof(crc_buffer));
				
				CRC_CalculateCont(&menu.rom_crc, crc_buffer, part);
				read_pos += part;
			}
			CRC_Finalize(&menu.rom_crc);
			menu.is_crc_checked = true;
		}
        
        if (is_rom_changed)
        {
            menu.selected_config = menu.cart_config.rom_configs[menu.selected_index];
            cart_set_mapping(menu.selected_config.mapping_regs);
            cart_set_save_offset(menu.selected_config.save_offset);
            data_cache_hit_writeback_invalidate(cartData, sizeof(cartData));
            dma_read(cartData, 0, sizeof(cartData));
            data_cache_hit_invalidate(cartData, sizeof(cartData));
            bootCodeCrc = CRC_Calculate(&cartData[16], 4032);
            menu.is_crc_checked = false;
        }
        
        if (timer_ticks() - tick > TIMER_TICKS(500000))
        {
            tick = timer_ticks();
            blink = !blink;
        }
   
        /*Fill the screen */
        graphics_fill_screen( menu.disp, 0 );

        /* Set the text output color */
        graphics_set_color( 0xFFFFFFFF, 0 );
        
        sprintf(sStr, "Build = %s %s", __DATE__, __TIME__);
        graphics_draw_text( menu.disp, 20, 15, sStr);
        sprintf(sStr, "Menu v%d.%d.%d / FPGA [%s] v%d.%d.%d",
            MAJOR_VERSION, MINOR_VERSION, DEBUG_VERSION,
            get_cart_hw_str(menu.CartVersion.Id), menu.CartVersion.Major, menu.CartVersion.Minor, menu.CartVersion.Debug);
        graphics_draw_text( menu.disp, 20, 25, sStr);
        graphics_draw_text( menu.disp, 70, 40, "==== Brutzelkarte ====" );
        
        if (menu.cart_config.num_roms > 1)
        {
            sprintf(sStr, "Title: %d/%d", menu.selected_index + 1, menu.cart_config.num_roms);
            graphics_draw_text( menu.disp, 20, 50, sStr );
        }
        
        sprintf(sStr, " Name: %.20s", cartInfo->ImageName);
        graphics_draw_text( menu.disp, 20, 60, sStr );
        
        if (memcmp(&cartInfo->CartridgeId[1], menu.selected_config.id, 2) == 0)
        {
            sprintf(sStr, "   ID: %.4s", cartInfo->CartridgeId);
            graphics_draw_text( menu.disp, 20, 70, sStr );
        }
        else
        {
            // print id as red text if it doesn't match'the config
            graphics_set_color( 0xFF0000FF, 0 );
            sprintf(sStr, "   ID: %.4s (%.2s)", cartInfo->CartridgeId, menu.selected_config.id);
            graphics_draw_text( menu.disp, 20, 70, sStr );
            graphics_set_color( 0xFFFFFFFF, 0 );
        }
        
        sprintf(sStr, "  CFG: %s", menu.selected_config.name);
        if (strlen(sStr) > 37)
        {
            strcpy(&sStr[34], "...");
        }
        graphics_draw_text( menu.disp, 20, 80, sStr );
        
        int8_t cart_tv = get_tv_type_by_id(cartInfo->CartridgeId[3]);
        if (cart_tv < 0)
        {
            // unknown TV -> yellow
            graphics_set_color( 0xFFFF00FF, 0 );
        }
        else if (cart_tv != menu.selected_config.tv)
        {
            // TV mismatch -> red
            graphics_set_color( 0xFF0000FF, 0 );
        }
        
        // TV
        sprintf(sStr, "   TV: %s", get_tvtype_str(menu.selected_config.tv));
        graphics_draw_text( menu.disp, 20, 100, sStr );
        
        graphics_set_color( 0xFFFFFFFF, 0 );
        uint8_t cart_cic = get_cic(bootCodeCrc);
        if (cart_cic == E_CICTYPE_UNKNOWN)
        {
            // unknown CIC -> yellow
            graphics_set_color( 0xFFFF00FF, 0 );
        }
        else if (cart_cic != menu.selected_config.cic)
        {
            // CIC mismatch -> red
            graphics_set_color( 0xFF0000FF, 0 );
        }
        
        // CIC
        sprintf(sStr, "  CIC: %s", get_cic_str(menu.selected_config.cic));
        graphics_draw_text( menu.disp, 20, 110, sStr );
        
        graphics_set_color( 0xFFFFFFFF, 0 );
        
        // Save
        sprintf(sStr, " Save: %s", get_savetype_str(menu.selected_config.save));
        graphics_draw_text( menu.disp, 20, 120, sStr );
        
        uint32_t cursor_color = graphics_make_color(255, 255, 255, _CursorAnimationAlpha[menu.cursor_anim]);
        graphics_draw_box_trans( menu.disp, 0, 99 + menu.cursor_pos * 10, 320, 10, cursor_color);
        
        menu.cursor_anim++;
        if (menu.cursor_anim >= sizeof(_CursorAnimationAlpha)/sizeof(int))
            menu.cursor_anim = 0;
        
        if (blink)
        {
            if (menu.keysDown.c[0].err == ERROR_NONE)
            {
                if ((menu.selected_config.cic != E_CICTYPE_UNKNOWN)
                    && (menu.selected_config.tv >= 0))
                {
                    graphics_draw_text( menu.disp, 80, 140, "Press START" );
                }
                else
                {
                    graphics_set_color( 0xFF0000FF, 0 );
                    graphics_draw_text( menu.disp, 60, 140, "Incomplete setup" );
                    graphics_set_color( 0xFFFFFFFF, 0 );
                }
            }
            else
            {
                graphics_set_color( 0xFF0000FF, 0 );
                graphics_draw_text( menu.disp, 60, 140, "Controller 1 not found" );
                graphics_set_color( 0xFFFFFFFF, 0 );
            }
        }
        
        strcpy(sStr, "ROM CRC check : ");
        if (menu.is_crc_checked)
        {
			strcat(sStr, (menu.rom_crc == menu.selected_config.rom_crc) ? "OK" : "FAIL");
			sprintf(sStr + strlen(sStr), " (%08lX)", menu.rom_crc);
		}
        else
        {
			strcat(sStr, "Press L to check");
		}
        graphics_draw_text( menu.disp, 20, 170, sStr );
        
        sprintf(sStr, "ROM size: %ld", menu.selected_config.rom_size);
        graphics_draw_text( menu.disp, 20, 180, sStr );
        
        sprintf(sStr, "SAVE offset: %ld", menu.selected_config.save_offset);
        graphics_draw_text( menu.disp, 20, 190, sStr );
        
        sprintf(sStr, "Bootcode CRC: %08X", (unsigned int)bootCodeCrc);
        graphics_draw_text( menu.disp, 20, 200, sStr );
        
        if (rtc_present())
        {
            struct tm rtc_time;
            rtc_read(&rtc_time);
            sprintf(sStr, "%d-%02d-%02d %02d:%02d:%02d",
                rtc_time.tm_year,
                rtc_time.tm_mon + 1,
                rtc_time.tm_mday,
                rtc_time.tm_hour,
                rtc_time.tm_min,
                rtc_time.tm_sec
            );
            graphics_draw_text( menu.disp, 20, 210, sStr );
        }
        else
        {
            graphics_draw_text( menu.disp, 20, 210, "No RTC" );
        }
        
        if ((menu.keysDown.c[0].start)
            && (menu.selected_config.cic != E_CICTYPE_UNKNOWN)
            && (menu.selected_config.tv >= 0))
        {
            save_backup_data(&menu);
            cart_set_savetype(menu.selected_config.save);
            start_game(menu.selected_config.tv, menu.selected_config.cic);
            
            // should not get here
            while (1) ;
        }
        
        display_show(menu.disp);
    }
}
