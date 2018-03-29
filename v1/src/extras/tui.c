/**
 * Provides a minimal curses-like interface for writing to the terminal screen.
 */

#include <stdarg.h>

#include "API.h"
#include "tui.h"

/* This prototppe is not defined in API.h, but the PROS does define the
 * function, so it can be used.
 */
int vfprintf(PROS_FILE *stream, const char *format, va_list args);

inline void move(int row, int col) {
    /* Values are passed as 0-indexed but written as 1-indexed */
    printf(CURSOR_POS, row+1, col+1);
}

void mvaddch(int row, int col, int c) {
    move(row, col);
    addch(c);
}

void mvaddchstr(int row, int col, const char *str) {
    move(row, col);
    addchstr(str);
}

void mvprintf(int row, int col, const char *format, ...) {
    va_list ap;

    move(row, col);

    va_start(ap, format);
    vfprintf(stdout, format, ap);
    va_end(ap);
}

void erase(void) {
    addchstr(ERASE_DISPLAY CURSOR_HOME);
}

void vline(int c, int n) {
    for (int i = 0; i < n; i++) {
        addch(c);
        addchstr(CURSOR_DOWN CURSOR_LEFT);
        /* printf(CURSOR_DOWN_N CURSOR_LEFT_N, 1, 1); */
    }
}

void mvvline(int row, int col, int c, int n) {
    move(row, col);
    vline(c, n);
}

void hline(int c, int n) {
    for (int i = 0; i < n; i++) {
        addch(c);
    }
}

void mvhline(int row, int col, int c, int n) {
    move(row, col);
    hline(c, n);
}

/* vim: set tw=80 ft=c: */
