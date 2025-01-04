// FILE:    F28069M.cmd
//
// TITLE:   Linker Command File For F28069M + Flash28_API Device

MEMORY
{
PAGE 0 :   /* Program Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE1 for data allocation */
   RAML0_1     : origin = 0x008000, length = 0x000C00     /* on-chip RAM block L0 and L1 */
   OTP         : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */

   //FLASHH      : origin = 0x3D8000, length = 0x004000     /* on-chip FLASH */
   //FLASHG      : origin = 0x3DC000, length = 0x004000     /* on-chip FLASH */
   FLASHF      : origin = 0x3E0000, length = 0x004000     /* on-chip FLASH */
   //FLASHE      : origin = 0x3E4000, length = 0x004000     /* on-chip FLASH */
   FLASHA_D      : origin = 0x3E8000, length = 0x00FF80     /* on-chip FLASH */
   //FLASHC      : origin = 0x3EC000, length = 0x004000     /* on-chip FLASH */
   //FLASHA_B    : origin = 0x3EC000, length = 0x007F80     /* on-chip FLASH */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */

   FPUTABLES   : origin = 0x3FD590, length = 0x0006A0	 /* FPU Tables in Boot ROM */
   IQTABLES    : origin = 0x3FDC30, length = 0x000B50    /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FE780, length = 0x00008C    /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FE80C, length = 0x0000AA	 /* IQ Math Tables in Boot ROM */

   ROM         : origin = 0x3FF3B0, length = 0x000C10     /* Boot ROM */
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

PAGE 1 :   /* Data Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE0 for program allocation */
           /* Registers remain on PAGE1                                                  */

   BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAML2_3     : origin = 0x008C00, length = 0x001400     /* on-chip RAM block L2 */
   RAML4_5       : origin = 0x00A000, length = 0x004000     /* on-chip RAM block L4 */
   //RAML5       : origin = 0x00C000, length = 0x002000     /* on-chip RAM block L5 */
   RAML6_7       : origin = 0x00E000, length = 0x004000     /* on-chip RAM block L6 */
   //RAML7       : origin = 0x010000, length = 0x002000     /* on-chip RAM block L7 */
   RAML8       : origin = 0x012000, length = 0x001800     /* on-chip RAM block L8. From 0x13800 to 0x14000 is reserved for InstaSPIN */
   USB_RAM     : origin = 0x040000, length = 0x000800     /* USB RAM		  */

   ECANA         : origin = 0x006000, length = 0x000040     /* eCAN-A control and status registers */
   ECANA_LAM     : origin = 0x006040, length = 0x000040     /* eCAN-A local acceptance masks */
   ECANA_MOTS    : origin = 0x006080, length = 0x000040     /* eCAN-A message object time stamps */
   ECANA_MOTO    : origin = 0x0060C0, length = 0x000040     /* eCAN-A object time-out registers */
   ECANA_MBOX    : origin = 0x006100, length = 0x000100     /* eCAN-A mailboxes */

   FLASHH      : origin = 0x3D8000, length = 0x004000     /* on-chip FLASH */
   FLASHG      : origin = 0x3DC000, length = 0x004000     /* on-chip FLASH */
   FLASHE      : origin = 0x3E4000, length = 0x004000     /* on-chip FLASH */
}

SECTIONS
{
      /* Allocate program areas: */
   /* The Flash API functions can be grouped together as shown below.
      The defined symbols _Flash28_API_LoadStart, _Flash28_API_LoadEnd
      and _Flash28_API_RunStart are used to copy the API functions out
      of flash memory and into SARAM */

/******************************************************************/
/* For Piccolo we dont need to copy the API from Flash as it is */
/* present in BOOT ROM											  */
/******************************************************************/
   Flash28_API: // Applicable only when API is not in BootROM
   {
      -lFlash2806x_API_wFPU_Library.lib(.econst)
      -lFlash2806x_API_wFPU_Library.lib(.text)
   }                   LOAD = FLASHG,
                       RUN = RAML4_5,
                       LOAD_START(_Flash28_API_LoadStart),
                       LOAD_END(_Flash28_API_LoadEnd),
                       RUN_START(_Flash28_API_RunStart),
                       PAGE = 1

   /* Allocate program areas: */
   .cinit              : > FLASHA_D,   PAGE = 0
   .pinit              : > FLASHA_D,   PAGE = 0
   .text               : > FLASHA_D,   PAGE = 0
   codestart           : > BEGIN,      PAGE = 0
   ramfuncs            :
   //{
   //   -lFlash2806x_API_wFPU_Library.lib(.econst)
   //   -lFlash2806x_API_wFPU_Library.lib(.text)
   //}
                        LOAD = FLASHE,
                        RUN = RAML6_7, /*RAML0_1*/
                        LOAD_START(_RamfuncsLoadStart),
                        LOAD_END(_RamfuncsLoadEnd),
                        RUN_START(_RamfuncsRunStart),
                        PAGE = 1

   csmpasswds          : > CSM_PWL_P0, PAGE = 0
   csm_rsvd            : > CSM_RSVD,   PAGE = 0

   /* Allocate uninitalized data sections: */
   .stack              : > RAMM0,      PAGE = 1
   .ebss               : > RAML2_3,    PAGE = 1
   .esysmem            : > RAML2_3,    PAGE = 1

   /* Initalized sections to go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   .econst             : > FLASHA_D,   PAGE = 0
   .switch             : > FLASHA_D,   PAGE = 0

   /* Allocate IQ math areas: */
   IQmath              : > FLASHA_D,   PAGE = 0            /* Math Code */
   IQmathTables        : > IQTABLES,   PAGE = 0, TYPE = NOLOAD
   
   /* Allocate FPU math areas: */
   FPUmathTables       : > FPUTABLES,  PAGE = 0, TYPE = NOLOAD
   
   //DMARAML5	           : > RAML5,      PAGE = 1
   //DMARAML6	           : > RAML6,      PAGE = 1
   //DMARAML7	           : > RAML7,      PAGE = 1
   DMARAML8	           : > RAML8,      PAGE = 1   
   
   //DRONECAN	           : > RAML6,      PAGE = 1
  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }
   */
   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables3
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }
   */

   /* .reset is a standard section used by the compiler.  It contains the */
   /* the address of the start of _c_int00 for C Code.  */
   /* When using the boot ROM this section and the CPU vector */
   /* table is not needed.  Thus the default type is set here to  */
   /* DSECT  */
   .reset              : > RESET,      PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS,    PAGE = 0, TYPE = DSECT

   persistent_memory   : > FLASHH,   PAGE = 1
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
