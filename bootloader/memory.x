MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH                             : ORIGIN = 0x08000000, LENGTH = 64K
  RAM                         (rwx) : ORIGIN = 0x24000000, LENGTH = 524280 /* 512K - 8 */
  BACKUP_RAM                  (rw)  : ORIGIN = 0x2407fff8, LENGTH = 8
}

__app_address = ORIGIN(FLASH) + LENGTH(FLASH);

SECTIONS
{
    .backup_ram (NOLOAD):
    {
        *(.backup_ram)
    } > BACKUP_RAM
}
