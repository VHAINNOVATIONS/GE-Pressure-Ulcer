#include "extcode.h"
#pragma pack(push)
#pragma pack(1)

#ifdef __cplusplus
extern "C" {
#endif

int32_t __cdecl INIT_dll(char ComPort[], char InitResponse[], int32_t len);
int32_t __cdecl CONFIG_dll(char ComPort[], uint32_t TenParameters[], 
	uint8_t ByteArrOut[], int32_t len, int32_t len2);
int32_t __cdecl SCAN_dll(char ComPort[], uint8_t ByteArrOut[], 
	uint32_t *SCANMsec, int32_t len);

long __cdecl LVDLLStatus(char *errStr, int errStrLen, void *module);

#ifdef __cplusplus
} // extern "C"
#endif

#pragma pack(pop)

