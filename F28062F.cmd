// FILE:    F28062F_flash.cmd
//
// TITLE:   Linker Command File For F28062F  + Flash28_API Device

MEMORY
{
PAGE 0 :   /* Program Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE1 for data allocation */
   RAML0_1      : origin = 0x008000, length = 0x000C00     /* on-chip RAM block L0 and L1 */
   OTP          : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */

   FLASHA_G       : origin = 0x3EA000, length = 0x00BF80     /* on-chip FLASH */
   CSM_RSVD     : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   BEGIN        : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   CSM_PWL_P0   : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */

   FPUTABLES     : origin = 0x3FD590, length = 0x0006A0	 /* FPU Tables in Boot ROM */
   IQTABLES      : origin = 0x3FDC30, length = 0x000B50    /* IQ Math Tables in Boot ROM */
   IQTABLES2     : origin = 0x3FE780, length = 0x00008C    /* IQ Math Tables in Boot ROM */
   IQTABLES3     : origin = 0x3FE80C, length = 0x0000AA	 /* IQ Math Tables in Boot ROM */

   ROM           : origin = 0x3FF3B0, length = 0x000C10     /* Boot ROM */
   RESET         : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
   VECTORS       : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

PAGE 1 :   /* Data Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE0 for program allocation */
           /* Registers remain on PAGE1                                                  */

   BOOT_RSVD     : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0_1       : origin = 0x000050, length = 0x0007B0     /* on-chip RAM block M0 */
   RAML2_3_4_5   : origin = 0x008C00, length = 0x005400
   USB_RAM       : origin = 0x040000, length = 0x000800     /* USB RAM		  */

   ECANA         : origin = 0x006000, length = 0x000040     /* eCAN-A control and status registers */
   ECANA_LAM     : origin = 0x006040, length = 0x000040     /* eCAN-A local acceptance masks */
   ECANA_MOTS    : origin = 0x006080, length = 0x000040     /* eCAN-A message object time stamps */
   ECANA_MOTO    : origin = 0x0060C0, length = 0x000040     /* eCAN-A object time-out registers */
   ECANA_MBOX    : origin = 0x006100, length = 0x000100     /* eCAN-A mailboxes */
   
   FLASHH        : origin = 0x3E8000, length = 0x002000     /* Persistent memory in Flash */
}

SECTIONS
{
   /* Allocate program areas: */
   //.cio                : > RAML0_1,        PAGE = 0
   .cinit              : > FLASHA_G,       PAGE = 0
   .pinit              : > FLASHA_G,       PAGE = 0
   .text               : > FLASHA_G,       PAGE = 0
   codestart           : > BEGIN,          PAGE = 0
   ramfuncs            : 
   {
        -lFlash2806x_API_wFPU_Library.lib(.econst)
        -lFlash2806x_API_wFPU_Library.lib(.text)
   }
                        LOAD = FLASHA_G,
                        RUN = RAML0_1,
                        LOAD_START(_RamfuncsLoadStart),
                        LOAD_END(_RamfuncsLoadEnd),
                        RUN_START(_RamfuncsRunStart),
                        PAGE = 0

   csmpasswds          : > CSM_PWL_P0,       PAGE = 0
   csm_rsvd            : > CSM_RSVD,         PAGE = 0

   /* Allocate uninitalized data sections: */
   .stack              : > RAMM0_1,          PAGE = 1
   .esysmem            : > RAML2_3_4_5,      PAGE = 1
   .ebss               : > RAML2_3_4_5,      PAGE = 1

   /* Initalized sections to go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   .econst             : > FLASHA_G,    PAGE = 0
   .switch             : > FLASHA_G,    PAGE = 0

   /* Allocate IQ math areas: */
   IQmath              : > FLASHA_G,    PAGE = 0            /* Math Code */
   IQmathTables        : > IQTABLES,    PAGE = 0, TYPE = NOLOAD
   
   /* Allocate FPU math areas: */
   FPUmathTables       : > FPUTABLES,   PAGE = 0, TYPE = NOLOAD

   DMARAML5	           : > RAML2_3_4_5, PAGE = 1

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
      is 1 wait-state). If this section is not uncommented, IQmathTables2
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
   /* the address of the start of _c_int00 for C Code.   /*
   /* When using the boot ROM this section and the CPU vector */
   /* table is not needed.  Thus the default type is set here to  */
   /* DSECT  */
   .reset              : > RESET,      PAGE = 0, TYPE = DSECT
   vectors             : > VECTORS,    PAGE = 0, TYPE = DSECT

   persistent_memory   : > FLASHH,     PAGE = 1
}
/*
//===========================================================================
// End of file.
//===========================================================================
*/
