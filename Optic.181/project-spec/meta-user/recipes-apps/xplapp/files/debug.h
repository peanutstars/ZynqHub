#include <stdio.h>


#define TAG_DBG     "[D] "
#define TAG_ERR     "[E] "

#define DBG(x,args...)      printf(TAG_DBG x, ##args)
#define ERR(x,args...)      printf(TAG_ERR x, ##args)


#define NOFD        (-1)
#define IERROR      (-1)
#define IOK         (0)
