/*

	GOLD -		Заводская прошивка (которая прошивается программатором)
	FIRMWARE -	Передаваемая прошивка (по COM-порту)
	FLASH_API -	Загрузчик, который проверяет прошивки при старте платы

	Для подготовки FIRMWARE:
		Поменять адрес BEGIN на 0x0A0000
		Поменять адрес FLASH_TO_WRITE на 0x0A0002
		Прошить проект во FLASH и выгрузить память

	Для загрузки GOLD:
		Поменять адрес BEGIN на 0x0B0000
		Поменять адрес FLASH_TO_WRITE на 0x0B0002
		Прошить проект FLASH
		Прошить в микроконтроллер проект FLASH_API, стирая только необходимые секторы

	Для старта во FLASH отсюда:
		Поменять адрес BEGIN на 0x080000
		Поменять адрес FLASH_TO_WRITE на 0x0B0002
		Прошить проект во FLASH без возможности перепрошивки через SCI



*/

MEMORY
{
PAGE 0 :

   /* BEGIN is used for the "boot to SARAM" bootloader mode   */



   BEGIN       : origin = 0x0B0000, length = 0x000002
   //BEGIN       : origin = 0x0A0000, length = 0x000002
   //BEGIN       : origin = 0x080000, length = 0x000002


    RAMLS0LS1LS2LS3LS4LS5     : origin = 0x008000, length = 0x003000
  // RAMLS2      : origin = 0x009000, length = 0x000800
   RESET       : origin = 0x3FFFC0, length = 0x000002
 //  IQTABLES    : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
  // IQTABLES2   : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
  // IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA	  /* IQ Math Tables in Boot ROM */

   /* Flash sectors */
   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   //FLASH_TO_WRITE   : origin = 0x0A0002, length = 0x007FFE	/* on-chip Flash */
   FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASH_TO_WRITE   : origin = 0x0B0002, length = 0x007FFE	/* on-chip Flash */
   FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASHN           : origin = 0x0BE000, length = 0x002000	/* on-chip Flash */


   BOOTROM     : origin = 0x3FF27C, length = 0x000D44


PAGE 1 :

   RAMM0       : origin = 0x000004, length = 0x000250
   RAMM1       : origin = 0x000400, length = 0x000250     /* on-chip RAM block M1 */
   RAMD0       : origin = 0x00B000, length = 0x000250
   RAMD1       : origin = 0x00B800, length = 0x000250

   RAMLS3      : origin = 0x009800, length = 0x000800
   RAMLS4LS5   : origin = 0x00A000, length = 0x001000


   RAMGS0GS1   : origin = 0x00C000, length = 0x002000
   RAMGS2      : origin = 0x00E000, length = 0x001000
   RAMGS3      : origin = 0x00F000, length = 0x001000

/*   RAMGS4      : origin = 0x010000, length = 0x001000
   RAMGS5      : origin = 0x011000, length = 0x001000
   RAMGS6      : origin = 0x012000, length = 0x001000
   RAMGS7      : origin = 0x013000, length = 0x001000*/


   /*
   RAMGS8      : origin = 0x014000, length = 0x001000
   RAMGS9      : origin = 0x015000, length = 0x001000
   RAMGS10     : origin = 0x016000, length = 0x001000
   RAMGS11     : origin = 0x017000, length = 0x001000
   RAMGS12     : origin = 0x018000, length = 0x001000
   RAMGS13     : origin = 0x019000, length = 0x001000
   RAMGS14     : origin = 0x01A000, length = 0x001000
   RAMGS15     : origin = 0x01B000, length = 0x001000
   */
   RAMGS8_13 	: origin = 0x014000, length = 0x006000
   RAMGS14		: origin = 0x01A000, length = 0x001000
   RAMGS15		: origin = 0x01B000, length = 0x001000

   FLASHA      : origin = 0x80000, length = 0x002000      /* on-chip FLASH */
   //FLASHB      : origin = 0x82000, length = 0x002000      /* on-chip FLASH */

}


SECTIONS
{
   /* Setup for "boot to SARAM" mode:
      The codestart section (found in DSP28_CodeStartBranch.asm)
      re-directs execution to the start of user code.  */
   codestart        : > BEGIN,     PAGE = 0
   .TI.ramfunc         : LOAD = FLASH_TO_WRITE,
                      RUN = RAMLS0LS1LS2LS3LS4LS5 ,
                      LOAD_START(_RamfuncsLoadStart),
                      LOAD_SIZE(_RamfuncsLoadSize),
                      LOAD_END(_RamfuncsLoadEnd),
                      RUN_START(_RamfuncsRunStart),
                      RUN_SIZE(_RamfuncsRunSize),
                      RUN_END(_RamfuncsRunEnd),
                      PAGE = 0
   .text            : > FLASH_TO_WRITE,    PAGE = 0
   .cinit           : > FLASH_TO_WRITE,     PAGE = 0
   .pinit           : > FLASH_TO_WRITE,     PAGE = 0
   .switch          : > FLASH_TO_WRITE,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */



   .stack           : > RAMLS3,     PAGE = 1
   .ebss            : > RAMGS0GS1,     PAGE = 1
   .econst          : > FLASH_TO_WRITE,     PAGE = 0
   .esysmem         : > RAMGS0GS1,     PAGE = 1

 /*GROUP : > CPU1TOCPU2RAM, PAGE = 1
    {
        PUTBUFFER
        PUTWRITEIDX
        GETREADIDX
    }

    GROUP : > CPU2TOCPU1RAM, PAGE = 1
    {
        GETBUFFER :    TYPE = DSECT
        GETWRITEIDX :  TYPE = DSECT
        PUTREADIDX :   TYPE = DSECT
    }
*/
   /* Allocate IQ math areas: */
   IQmath			: > FLASH_TO_WRITE, PAGE = 0, ALIGN(4)            /* Math Code */
   IQmathTables		: > FLASH_TO_WRITE, PAGE = 0, ALIGN(4)
   
   FirmwareBufferSection   : > RAMGS8_13,     PAGE = 1
   FirmwareLengthSection   : > RAMGS14,     PAGE = 1
   FirmwareSumSection   : > RAMGS15,     PAGE = 1


   //FirmwareBuffer		: >> RAMGS2|RAMGS3|RAMGS4|RAMGS5|RAMGS6|RAMGS7, PAGE = 1
  /* IQmath           : > RAML0,     PAGE = 0
   IQmathTables     : > IQTABLES,  PAGE = 0, TYPE = NOLOAD*/
  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */

  /* IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }*/

   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */

 /*  IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }*/


}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
