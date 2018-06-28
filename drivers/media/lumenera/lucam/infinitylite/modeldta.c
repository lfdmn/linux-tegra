#include "../lucam.h"



static const unsigned char model_data_03ad[] = {
1, 0 , 0 ,0 ,100,0,0,0,
1,0,
146,17,146,253,220,0,215,0,88,14,207,0,38,2,246,254,228,14,
2,0,
51,25,201,241,4,5,21,253,138,20,97,254,142,2,114,245,255,23,
3,0,
144,27,116,243,251,0,121,255,20,17,117,255,164,0,235,242,113,28,
4,0,
49,24,199,245,7,2,230,0,203,15,78,255,132,0,39,255,85,16,
5,0,
51,50,102,232,103,245,143,249,112,35,2,243,5,251,177,255,75,21,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity Lite
   {
      0x03ad,
      0xffff,
      model_data_03ad,
      sizeof(model_data_03ad),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


