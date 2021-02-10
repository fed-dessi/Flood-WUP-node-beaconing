/***************************************************************************//**
 * @brief RAIL Configuration
 * @details
 *   WARNING: Auto-Generated Radio Config  -  DO NOT EDIT
 *   Radio Configurator Version: 5.10.1
 *   RAIL Adapter Version: 2.4.13
 *   RAIL Compatibility: 2.x
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_device.h"
#include "rail_config.h"

uint32_t RAILCb_CalcSymbolRate(RAIL_Handle_t railHandle)
{
  (void) railHandle;
  return 0U;
}

uint32_t RAILCb_CalcBitRate(RAIL_Handle_t railHandle)
{
  (void) railHandle;
  return 0U;
}

void RAILCb_ConfigFrameTypeLength(RAIL_Handle_t railHandle,
                                  const RAIL_FrameType_t *frameType)
{
  (void) railHandle;
  (void) frameType;
}

static const uint8_t irCalConfig_0[] = {
  25, 63, 1, 6, 4, 16, 1, 0, 0, 1, 1, 6, 0, 16, 39, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint8_t irCalConfig_1[] = {
  25, 69, 3, 6, 4, 16, 0, 1, 1, 3, 1, 6, 0, 16, 39, 0, 0, 5, 0, 1, 1, 0, 0, 0, 0, 0
};

static RAIL_ChannelConfigEntryAttr_t channelConfigEntryAttr_0 = {
  { 0xFFFFFFFFUL }
};

static RAIL_ChannelConfigEntryAttr_t channelConfigEntryAttr_1 = {
  { 0xFFFFFFFFUL }
};

static const uint32_t phyInfo_0[] = {
  8UL,
  0x001D41D4UL, // 29.257142857142853
  (uint32_t) NULL,
  (uint32_t) irCalConfig_0,
#ifdef RADIO_CONFIG_ENABLE_TIMING
  (uint32_t) &Channel_Group_1_timing,
#else
  (uint32_t) NULL,
#endif
  0x00000000UL,
  10000000UL,
  35000000UL,
  250000UL,
  (1UL << 8) | 1UL,
  0x020013C2UL,
  (uint32_t) NULL,
  (uint32_t) NULL,
  (uint32_t) NULL,
};

static const uint32_t phyInfo_1[] = {
  8UL,
  0x001135C8UL, // 17.210084033613445
  (uint32_t) NULL,
  (uint32_t) irCalConfig_1,
#ifdef RADIO_CONFIG_ENABLE_TIMING
  (uint32_t) &Channel_Group_2_timing,
#else
  (uint32_t) NULL,
#endif
  0x00000000UL,
  10200000UL,
  35700000UL,
  50000UL,
  (1UL << 8) | 1UL,
  0x0600231CUL,
  (uint32_t) NULL,
  (uint32_t) NULL,
  (uint32_t) NULL,
};

const uint32_t Protocol_Configuration_modemConfigBase[] = {
  0x00020004UL, 0x00000000UL,
  /*    0008 */ 0x00000000UL,
  0x00020018UL, 0x0000000FUL,
  /*    001C */ 0x00000000UL,
  0x00040028UL, 0x00000000UL,
  /*    002C */ 0x00000000UL,
  /*    0030 */ 0x00000000UL,
  /*    0034 */ 0x00000000UL,
  0x00010048UL, 0x00000000UL,
  0x00020054UL, 0x00000000UL,
  /*    0058 */ 0x00000000UL,
  0x000400A0UL, 0x00004CFFUL,
  /*    00A4 */ 0x00000000UL,
  /*    00A8 */ 0x00004DFFUL,
  /*    00AC */ 0x00000000UL,
  0x00012000UL, 0x00000744UL,
  0x00012010UL, 0x00000000UL,
  0x00012018UL, 0x0000A001UL,
  0x00013008UL, 0x0100AC13UL,
  0x00013040UL, 0x00000000UL,
  0x000140F4UL, 0x00001020UL,
  0x00044160UL, 0x00000000UL,
  /*    4164 */ 0x00000000UL,
  /*    4168 */ 0x00000006UL,
  /*    416C */ 0x00000006UL,
  0x00086014UL, 0x00000010UL,
  /*    6018 */ 0x04000000UL,
  /*    601C */ 0x0001400FUL,
  /*    6020 */ 0x00005000UL,
  /*    6024 */ 0x00000000UL,
  /*    6028 */ 0x03000000UL,
  /*    602C */ 0x00000000UL,
  /*    6030 */ 0x00000000UL,
  0x0003605CUL, 0x00140012UL,
  /*    6060 */ 0x0000B16FUL,
  /*    6064 */ 0x00000000UL,
  0x00026078UL, 0x08E0081FUL,
  /*    607C */ 0x00000000UL,
  0x00096084UL, 0x00000000UL,
  /*    6088 */ 0x00000000UL,
  /*    608C */ 0x22140A04UL,
  /*    6090 */ 0x4F4A4132UL,
  /*    6094 */ 0x00000000UL,
  /*    6098 */ 0x00000000UL,
  /*    609C */ 0x00000000UL,
  /*    60A0 */ 0x00000000UL,
  /*    60A4 */ 0x00000000UL,
  0x000660E8UL, 0x00000000UL,
  /*    60EC */ 0x07830464UL,
  /*    60F0 */ 0x3AC81388UL,
  /*    60F4 */ 0x0006209CUL,
  /*    60F8 */ 0x00206100UL,
  /*    60FC */ 0x208556B7UL,
  0x00026108UL, 0x00003020UL,
  /*    610C */ 0x0000BB88UL,
  0x00016120UL, 0x00000000UL,
  0x10017014UL, 0x0007F800UL,
  0x30017014UL, 0x000000FEUL,
  0x00017018UL, 0x00000300UL,
  0xFFFFFFFFUL,
};

const uint32_t Channel_Group_1_modemConfig[] = {
  0x01011FF8UL, (uint32_t) &phyInfo_0,
  0x01021FF0UL, 0x0037003FUL,
  /*    1FF4 */ 0x00000000UL,
  0x01011FFCUL, 0x00000000UL,
  0x00030038UL, 0x00000000UL,
  /*    003C */ 0x00000000UL,
  /*    0040 */ 0x00000700UL,
  0x00023030UL, 0x001013C2UL,
  /*    3034 */ 0x00000001UL,
  0x000140A0UL, 0x0F00277AUL,
  0x000140B8UL, 0x0083C000UL,
  0x00024134UL, 0x00000880UL,
  /*    4138 */ 0x000087E6UL,
  0x00024140UL, 0x008800A0UL,
  /*    4144 */ 0x4D52E6C1UL,
  0x00036050UL, 0x00FF1320UL,
  /*    6054 */ 0x00000C41UL,
  /*    6058 */ 0x00000022UL,
  0x00016080UL, 0x003B03B1UL,
  0x000160E4UL, 0x8C5B0881UL,
  0x00016104UL, 0x00109E0FUL,
  0x0005701CUL, 0x834A0060UL,
  /*    7020 */ 0x00000000UL,
  /*    7024 */ 0x00000082UL,
  /*    7028 */ 0x01800000UL,
  /*    702C */ 0x000000D5UL,
  0x00027048UL, 0x00003D3CUL,
  /*    704C */ 0x000019BCUL,
  0x00037070UL, 0x00120105UL,
  /*    7074 */ 0x00083027UL,
  /*    7078 */ 0x00552300UL,
  0xFFFFFFFFUL,
};

const uint32_t Channel_Group_2_modemConfig[] = {
  0x01011FF8UL, (uint32_t) &phyInfo_1,
  0x01021FF0UL, 0x003F003FUL,
  /*    1FF4 */ 0x00000000UL,
  0x01011FFCUL, 0x00000000UL,
  0x00030038UL, 0x00000000UL,
  /*    003C */ 0x00000000UL,
  /*    0040 */ 0x00000700UL,
  0x00023030UL, 0x0010231CUL,
  /*    3034 */ 0x00000003UL,
  0x000140A0UL, 0x0F0027AAUL,
  0x000140B8UL, 0x0023C000UL,
  0x00024134UL, 0x00000880UL,
  /*    4138 */ 0x000087F6UL,
  0x00024140UL, 0x00880020UL,
  /*    4144 */ 0x4D52E6C1UL,
  0x00036050UL, 0x00FF5FA0UL,
  /*    6054 */ 0x00000C41UL,
  /*    6058 */ 0x00000108UL,
  0x00016080UL, 0x003B0390UL,
  0x000160E4UL, 0x8C56087FUL,
  0x00016104UL, 0x00104637UL,
  0x0005701CUL, 0x830A0060UL,
  /*    7020 */ 0x00000000UL,
  /*    7024 */ 0x00000082UL,
  /*    7028 */ 0x00000000UL,
  /*    702C */ 0x000000D5UL,
  0x00027048UL, 0x0000383EUL,
  /*    704C */ 0x000025BCUL,
  0x00037070UL, 0x00120105UL,
  /*    7074 */ 0x00083031UL,
  /*    7078 */ 0x006D8480UL,
  0xFFFFFFFFUL,
};

const RAIL_ChannelConfigEntry_t Protocol_Configuration_channels[] = {
  {
    .phyConfigDeltaAdd = Channel_Group_1_modemConfig,
    .baseFrequency = 2450000000,
    .channelSpacing = 1000000,
    .physicalChannelOffset = 0,
    .channelNumberStart = 0,
    .channelNumberEnd = 0,
    .maxPower = RAIL_TX_POWER_MAX,
    .attr = &channelConfigEntryAttr_0,
#ifdef RADIO_CONFIG_ENABLE_CONC_PHY
    .entryType = 0
#endif
  },
  {
    .phyConfigDeltaAdd = Channel_Group_2_modemConfig,
    .baseFrequency = 868000000,
    .channelSpacing = 0,
    .physicalChannelOffset = 1,
    .channelNumberStart = 1,
    .channelNumberEnd = 1,
    .maxPower = RAIL_TX_POWER_MAX,
    .attr = &channelConfigEntryAttr_1,
#ifdef RADIO_CONFIG_ENABLE_CONC_PHY
    .entryType = 0
#endif
  },
};

const RAIL_ChannelConfig_t Protocol_Configuration_channelConfig = {
  .phyConfigBase = Protocol_Configuration_modemConfigBase,
  .phyConfigDeltaSubtract = NULL,
  .configs = Protocol_Configuration_channels,
  .length = 2U,
  .signature = 0UL,
};

const RAIL_ChannelConfig_t *channelConfigs[] = {
  &Protocol_Configuration_channelConfig,
  NULL
};

uint32_t protocolAccelerationBuffer[119];
