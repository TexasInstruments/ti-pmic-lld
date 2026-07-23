/******************************************************************************
 * syscalls.c - Minimal newlib syscall stubs for bare-metal ARM builds
 *
 * These are required by newlib's libc when building for embedded targets.
 * They provide minimal implementations for system calls used by standard
 * library functions like printf, malloc, etc.
 *****************************************************************************/

#include <sys/stat.h>
#include <errno.h>

/* Increase heap pointer */
void *_sbrk(int incr)
{
    extern char _end;  /* Defined by linker */
    static char *heap_end = 0;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;
    heap_end += incr;

    return (void *)prev_heap_end;
}

/* Write to a file - redirect to UART */
int _write(int file, char *ptr, int len)
{
    extern void platform_printChar(char c);
    int i;

    (void)file;  /* Ignore file descriptor */

    for (i = 0; i < len; i++) {
        platform_printChar(ptr[i]);
    }

    return len;
}

/* Unbuffered putchar for bare-metal console output */
int putchar(int c)
{
    extern void platform_printChar(char c);
    platform_printChar((char)c);
    return (unsigned char)c;
}

/* Close a file */
int _close(int file)
{
    (void)file;
    return -1;
}

/* Get file status */
int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;  /* Character device */
    return 0;
}

/* Check if file descriptor is a terminal */
int _isatty(int file)
{
    (void)file;
    return 1;  /* Assume all file descriptors are TTY */
}

/* Seek within a file */
int _lseek(int file, int offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}

/* Read from a file */
int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;  /* No input for now */
}

/* Exit program */
void _exit(int status)
{
    (void)status;
    /* Hang in infinite loop */
    while (1) {
        ;
    }
}

/* Send signal to process */
int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

/* Get process ID */
int _getpid(void)
{
    return 1;
}
