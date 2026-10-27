/*
 * Large application buffers go here, at the END of .noinit.  The SAM E5x CAN
 * controller reaches its message RAM (a __nocache_noinit buffer in the driver)
 * through 16-bit offsets, so that buffer must stay in the first 64 KiB of SRAM;
 * anything big linked before it pushes it out and CAN silently stops working.
 */
*(.dm_trace)
*(".dm_trace.*")
