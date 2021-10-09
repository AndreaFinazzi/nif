#ifndef H_CSC_COMMAND_LUMNET_DEFINITIONS_H
#define H_CSC_COMMAND_LUMNET_DEFINITIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "LumNet/Command.h"

enum LegacyCommandProtocolSemVer
{
    COMMAND_PROTOCOL_SEM_VER_LATEST = 0
};

static inline struct LumNet_Version GetLegacyCommandProtocolSemVer( enum LegacyCommandProtocolSemVer ver )
{
    switch ( ver )
    {
    case COMMAND_PROTOCOL_SEM_VER_LATEST:
    default:
        return LumNet_GetCommandProtocolSemVer();
    }
}

#ifdef __cplusplus
}
#endif

#endif
