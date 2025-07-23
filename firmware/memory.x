MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH                             : ORIGIN = 0x08010000, LENGTH = 1984K
  RAM                         (rwx) : ORIGIN = 0x24000000, LENGTH = 511K
  BACKUP_RAM                        : ORIGIN = 0x2407fc00, LENGTH = 1K
}

SECTIONS
{
    .backup_ram (NOLOAD):
    {
        *(.backup_ram)
    } > BACKUP_RAM
}
