#include "../lucam.h"



static const unsigned char model_data_00a0_00a1[] = {
1, 0 , 0 ,0 ,100,0,0,0,
1,0,
158,16,193,254,160,0,145,1,184,13,182,0,237,1,4,254,14,16,
2,0,
1,19,75,251,179,1,202,252,157,19,153,255,180,3,164,248,167,19,
3,0,
144,27,116,243,251,0,121,255,20,17,117,255,164,0,235,242,113,28,
4,0,
49,24,199,245,7,2,230,0,203,15,78,255,132,0,39,255,85,16,
5,0,
235,27,243,244,35,255,164,0,122,19,226,251,18,255,128,254,110,18,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // InfinityX
   {
      0x00a0,
      0xffff,
      model_data_00a0_00a1,
      sizeof(model_data_00a0_00a1),
   },
   // Infinity1
   {
      0x00a1,
      0xffff,
      model_data_00a0_00a1,
      sizeof(model_data_00a0_00a1),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


