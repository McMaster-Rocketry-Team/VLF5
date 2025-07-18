MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH                             : ORIGIN = 0x08010000, LENGTH = 1984K
  RAM                         (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}