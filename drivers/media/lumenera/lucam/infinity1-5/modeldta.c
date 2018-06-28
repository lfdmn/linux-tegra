#include "../lucam.h"



static const unsigned char model_data_01ac[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
88,30,103,242,65,255,113,255,237,18,162,253,218,2,147,246,147,22,
2,0,
113,24,54,243,89,4,68,244,59,27,128,0,64,2,208,246,240,22,
5,0,
93,49,34,225,129,253,59,246,136,32,62,249,52,253,229,253,232,20,
0,0,0,0};



struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity1-5
   {
      0x01ac,
      0xffff,
      model_data_01ac,
      sizeof(model_data_01ac),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


