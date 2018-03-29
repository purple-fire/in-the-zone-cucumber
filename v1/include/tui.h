/**
 * Provides a minimal curses-like interface for writing to the terminal screen.
 *
 * TODO Implement these as inline functions/macros?
 */

#ifndef TUI_H_
#define TUI_H_

#define ERASE_DISPLAY   "\e[2J"
#define ERASE_LINE      "\e[K"

#define CURSOR_UP       "\e[A"
#define CURSOR_DOWN     "\e[B"
#define CURSOR_RIGHT    "\e[C"
#define CURSOR_LEFT     "\e[D"
#define CURSOR_UP_N     "\e[%dA"
#define CURSOR_DOWN_N   "\e[%dB"
#define CURSOR_RIGHT_N  "\e[%dC"
#define CURSOR_LEFT_N   "\e[%dD"
#define CURSOR_HOME     "\e[H"
#define CURSOR_POS      "\e[%d;%dH"
#define CURSOR_SAVE     "\e[s"
#define CURSOR_RESTORE  "\e[u"

/**
 * Moves the cursor.
 *
 * @param row, col Coordinates to move to.
 */
void move(int row, int col);

/**
 * Adds a character at the current cursor position.
 *
 * @param c Character to add.
 */
#define addch(c) putchar(c)

/**
 * Moves the cursor and adds a character to the screen.
 *
 * @param row, col Coordinates to move to. (0, 0) is the top-left of the screen.
 * @param c Character to add.
 */
void mvaddch(int row, int col, int c);

/**
 * Adds a string to the screen.
 *
 * @param str String to add.
 */
#define addchstr(str) print(str)

/**
 * Moves the cursor and adds a character to the screen.
 *
 * @param row, col Coordinates to move to.
 * @param c Character to add.
 */
void mvaddchstr(int row, int col, const char *str);

/**
 * Moves the cursor and adds a formatted output to the screen.
 *
 * @param row, col Coordinates to move to.
 * @param format Format string.
 * @param ... Arguments to the format string.
 */
void mvprintf(int row, int col, const char *format, ...);

/**
 * Erases the whole screen.
 */
void erase(void);

/**
 * Draws a vertical line towards the bottom of the screen @p n lines
 * long using the character @p c.
 *
 * @param c Character to use.
 * @param n Length of the line.
 */
void vline(int c, int n);

/**
 * Moves the cursor and draws a vertical line towards the bottom of the screen
 * @p n lines long using the character @p c.
 *
 * @param row, col Starting position of the line.
 * @param c Character to use.
 * @param n Length of the line.
 */
void mvvline(int row, int col, int c, int n);

/**
 * Draws a horizontal line towards the right of the screen @p n columns long
 * using the character @p c.
 *
 * @param c Character to use.
 * @param n Length of the line.
 */
void hline(int c, int n);

/**
 * Moves the cursor and draws a horizontal line towards the right of the screen
 * at most @p n columns long using the character @p c.
 *
 * @param row, col Starting position of the line.
 * @param c Character to use.
 * @param n Length of the line.
 */
void mvhline(int row, int col, int c, int n);

#endif /* TUI_H_ */

/* vim: set tw=80 ft=c: */
