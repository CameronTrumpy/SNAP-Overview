/*
 * syscalls.c
 *
 *  Created on: Oct 11, 2025
 *      Author: Ethan
 */


#include <sys/stat.h>
#include <unistd.h>

int _close(int file) { return -1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return 1; }
int _write(int file, char *ptr, int len) { return len; }
pid_t _getpid(void) {return 0;}
int _kill(pid_t pid, int sig) {return 0;}
