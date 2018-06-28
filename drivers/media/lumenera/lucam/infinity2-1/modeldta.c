#include "../lucam.h"


// Infinity2-1R
static const unsigned char model_data_01a2_0018[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
234,18,36,251,241,1,27,255,23,18,207,254,7,4,159,249,89,18,
2,0,
141,21,214,249,157,0,32,251,127,23,99,253,43,255,95,251,120,21,
3,0,
33,22,137,248,85,1,92,254,20,20,145,253,68,4,184,246,4,21,
5,0,
108,18,148,252,0,1,249,252,175,20,92,254,169,254,167,254,178,18,
0,0,0,0
};


static const unsigned char model_data_01a2_ffff[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
234,18,35,251,242,1,26,255,24,18,206,254,7,4,159,249,89,18,
2,0,
127,18,186,251,199,1,247,250,159,20,105,0,62,4,202,249,247,17,
5,0,
167,16,217,252,127,2,50,2,22,15,184,254,134,0,165,0,212,14,
//LUMEMSGL_MODEL_DTA_SSBIN_DESC
3,0,0,0, 0x14,0,0,0
,0 //flags
,0 //reserved
,1 //ssNot1Count
,3//binNot1Count
,4,0,0,0,0,0,0,0 //ssFormatasNot1[8]
,2,3,4,0,0,0,0,0 //binFormatasNot1[8]

,0,0,0,0
};



struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity2-1R
   {
      0x01a2,
      0x0018,
      model_data_01a2_0018,
      sizeof(model_data_01a2_0018),
   },
   // Infinity2-1
   {
      0x01a2,
      0xffff,
      model_data_01a2_ffff,
      sizeof(model_data_01a2_ffff),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


