#pragma once

#define SC25_COB_ESC_HZ                                     (0x200)
#define SC25_CMD_ESC_HZ                                     (0x20)

#define SC25_COB_MODULO_COUNT                               (0x300)
#define SC25_CMD_MODULO_COUNT                               (0x32)

#define SC25_COB_DFC_ROT                                    (0x200)
#define SC25_CMD_DFC_ROT                                    (0x40)

#define SC25_COB_DFC_POS                                    (0x200)
#define SC25_CMD_DFC_POS                                    (0x44)

#define SC25_COB_RESET                                      (0x200)
#define SC25_CMD_RESET                                      (0x01)

#define SC25_COB_RESET_WORKZONE                             (0x200)
#define SC25_CMD_RESET_WORKZONE                             (0xE0)

#define SC25_COB_STOP                                       (0x200)
#define SC25_CMD_STOP                                       (0x04)

#define SC25_COB_READ                                       (0x600)
#define SC25_CMD_READ                                       (0x40)

#define SC25_COB_WRITE                                      (0x600)
#define SC25_CMD_WRITE                                      (0x20)

#define SC25_TELEMETRY_INDEX_DFCPOS                         (0x4008)
#define SC25_TELEMETRY_SUB_INDEX_DFCPOS                     (0x03)

#define SC25_TELEMETRY_INDEX_QUADRATURE_COUNT               (0x500A)
#define SC25_TELEMETRY_SUB_INDEX_QUADRATURE_COUNT           (0x06)

#define SC25_TELEMETRY_INDEX_QUADRATURE_COUNT               (0x500A)
#define SC25_TELEMETRY_SUB_INDEX_QUADRATURE_COUNT           (0x06)

#define SC25_TELEMETRY_INDEX_MODULO_COUNT                   (0x4000)
#define SC25_TELEMETRY_SUB_INDEX_MODULO_COUNT               (0x1A)

#define SC25_CONFIG_INDEX_ENCODER_BIAS                      (0x3011)
#define SC25_CONFIG_SUB_INDEX_ENCODER_BIAS                  (0x03)

#define SC25_COB_READ_RESPONSE                              (0x580)
#define SC25_CMD_RESPONSE_4B                                (0x43)