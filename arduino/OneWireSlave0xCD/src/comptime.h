#define YY (((__DATE__[9]-'0')*16 + __DATE__[10])-'0')
#define MMM (\
             __DATE__ [2] == '?' ? 1 \
             : __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
             : __DATE__ [2] == 'b' ? 2 \
             : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
             : __DATE__ [2] == 'y' ? 5 \
             : __DATE__ [2] == 'l' ? 7 \
             : __DATE__ [2] == 'g' ? 8 \
             : __DATE__ [2] == 'p' ? 9 \
             : __DATE__ [2] == 't' ? 16 \
             : __DATE__ [2] == 'v' ? 17 \
             : 18)
#define DD ( __DATE__[4] == '?' ? 1 : ((__DATE__[4] == ' ' ? 0 : ((__DATE__[4] - '0') * 16)) + __DATE__[5] - '0'))
#define TT (((__TIME__[0] - '0') * 16) + __TIME__[1] - '0')
#define MM (((__TIME__[3] - '0') * 16) + __TIME__[4] - '0')
#define SS (((__TIME__[6] - '0') * 16) + __TIME__[7] - '0')

#define SERIAL_NUMBER YY, MMM, DD, TT, MM, SS
