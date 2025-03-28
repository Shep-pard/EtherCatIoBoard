#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>


static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "Outputs";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "OUT1";
static const char acName1600_02[] = "OUT2";
static const char acName1600_03[] = "OUT3";
static const char acName1A00[] = "Inputs";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "IN1";
static const char acName1A00_02[] = "IN2";
static const char acName1A00_03[] = "IN3";
static const char acName1A01[] = "Encoder";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "ENC1";
static const char acName1A01_02[] = "ENC2";
static const char acName1A01_03[] = "ENC3";
static const char acName1A01_04[] = "ENC4";
static const char acName1A02[] = "Analog";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "A1";
static const char acName1A02_02[] = "A2";
static const char acName1A02_03[] = "A3";
static const char acName1A02_04[] = "A4";
static const char acName1A02_05[] = "A5";
static const char acName1A02_06[] = "A6";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C13_03[] = "PDO Mapping";
static const char acName6000[] = "Inputs";
static const char acName6000_00[] = "Max SubIndex";
static const char acName6000_01[] = "IN1";
static const char acName6000_02[] = "IN2";
static const char acName6000_03[] = "IN3";
static const char acName6001[] = "Encoder";
static const char acName6001_00[] = "Max SubIndex";
static const char acName6001_01[] = "ENC1";
static const char acName6001_02[] = "ENC2";
static const char acName6001_03[] = "ENC3";
static const char acName6001_04[] = "ENC4";
static const char acName6002[] = "Analog";
static const char acName6002_00[] = "Max SubIndex";
static const char acName6002_01[] = "A1";
static const char acName6002_02[] = "A2";
static const char acName6002_03[] = "A3";
static const char acName6002_04[] = "A4";
static const char acName6002_05[] = "A5";
static const char acName6002_06[] = "A6";
static const char acName7001[] = "Outputs";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "OUT1";
static const char acName7001_02[] = "OUT2";
static const char acName7001_03[] = "OUT3";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 64, ATYPE_RO, acName1008, 0, "IO Board"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "0.0.1"},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "0.0.1"},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 4919, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 3735928559, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 1, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 1, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x70010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_02, 0x70010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_03, 0x70010310, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x60000310, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60010120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x60010220, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x60010320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x60010420, NULL},
};
const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 6, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_02, 0x60020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_03, 0x60020310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_04, 0x60020410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_05, 0x60020510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_06, 0x60020610, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, NULL},
};
const _objd SDO6000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6000_01, 0, &Obj.Inputs[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6000_02, 0, &Obj.Inputs[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6000_03, 0, &Obj.Inputs[2]},
};
const _objd SDO6001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6001_00, 4, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6001_01, 0, &Obj.Encoder[0]},
  {0x02, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6001_02, 0, &Obj.Encoder[1]},
  {0x03, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6001_03, 0, &Obj.Encoder[2]},
  {0x04, DTYPE_INTEGER32, 32, ATYPE_RO | ATYPE_TXPDO, acName6001_04, 0, &Obj.Encoder[3]},
};
const _objd SDO6002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6002_00, 6, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_01, 0, &Obj.Analog[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_02, 0, &Obj.Analog[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_03, 0, &Obj.Analog[2]},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_04, 0, &Obj.Analog[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_05, 0, &Obj.Analog[4]},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_06, 0, &Obj.Analog[5]},
};
const _objd SDO7001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7001_01, 0, &Obj.Outputs[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7001_02, 0, &Obj.Outputs[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7001_03, 0, &Obj.Outputs[2]},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 3, 0, acName1600, SDO1600},
  {0x1A00, OTYPE_RECORD, 3, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 4, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 6, 0, acName1A02, SDO1A02},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 1, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 3, 0, acName1C13, SDO1C13},
  {0x6000, OTYPE_ARRAY, 3, 0, acName6000, SDO6000},
  {0x6001, OTYPE_ARRAY, 4, 0, acName6001, SDO6001},
  {0x6002, OTYPE_ARRAY, 6, 0, acName6002, SDO6002},
  {0x7001, OTYPE_ARRAY, 3, 0, acName7001, SDO7001},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
