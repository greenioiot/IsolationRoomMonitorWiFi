// We need this header file to use FLASH as storage with PROGMEM directive:
#include <pgmspace.h>

// Icon width and height
const uint16_t wifi3Width = 32;
const uint16_t wifi3Height = 24;

const unsigned short wifi3[768] PROGMEM={
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x5ACB, 0xB5B6, 0xDEFB, 0xE71C, 0xE73C, 0xEF5D, 0xEF5D, 0xEF7D, 0xEF7D,   // 0x0010 (16) pixels
0xF79E, 0xEF7D, 0xEF5D, 0xEF5D, 0xE73C, 0xE73C, 0xDEFB, 0xBDD7, 0x6B4D, 0x0841, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0020 (32) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0020, 0x738E, 0xCE79, 0xDEFB, 0xE71C, 0xE71C, 0xE73C, 0xE73C, 0xEF5D, 0xEF5D, 0xEF7D, 0xEF7D,   // 0x0030 (48) pixels
0xEF7D, 0xEF7D, 0xEF5D, 0xEF5D, 0xE73C, 0xE73C, 0xE71C, 0xDEFB, 0xDEFB, 0xD69A, 0x7BEF, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0040 (64) pixels
0x0000, 0x0000, 0x0000, 0x2104, 0xB5B6, 0xDEDB, 0xDEFB, 0xDEFB, 0xE71C, 0xE71C, 0xE73C, 0xE73C, 0xE73C, 0xE73C, 0xD71C, 0xD71C,   // 0x0050 (80) pixels
0xD71C, 0xD71C, 0xDF1C, 0xE73C, 0xE73C, 0xE71C, 0xE71C, 0xE71C, 0xDEFB, 0xDEDB, 0xDEDB, 0xBDF7, 0x31A6, 0x0000, 0x0000, 0x0000,   // 0x0060 (96) pixels
0x0000, 0x0000, 0x632C, 0xD6BA, 0xDEDB, 0xDEDB, 0xDEFB, 0xDEFB, 0xE71C, 0xDEFB, 0xA63B, 0x555A, 0x2519, 0x2497, 0x2C54, 0x2433,   // 0x0070 (112) pixels
0x2413, 0x2434, 0x2CB7, 0x251A, 0x4D5A, 0x9E1B, 0xD6DB, 0xDEFB, 0xDEFB, 0xDEDB, 0xDEDB, 0xD6BA, 0xD6BA, 0x7BCF, 0x0000, 0x0000,   // 0x0080 (128) pixels
0x0020, 0x7BCF, 0xD69A, 0xD6BA, 0xDEDB, 0xDEDB, 0xDEDB, 0xCEBB, 0x7D9A, 0x23F5, 0x2B71, 0x222A, 0x1167, 0x10E4, 0x10C3, 0x10A2,   // 0x0090 (144) pixels
0x0882, 0x08A3, 0x1105, 0x19A7, 0x226C, 0x2331, 0x1BD6, 0x7559, 0xC69A, 0xDEDB, 0xDEDB, 0xD6BA, 0xD6BA, 0xD69A, 0x9492, 0x0841,   // 0x00A0 (160) pixels
0x9CD3, 0xD69A, 0xD69A, 0xD6BA, 0xDEDB, 0xD6BB, 0x7D59, 0x2BF7, 0x3330, 0x1946, 0x0000, 0x0000, 0x0020, 0x1924, 0x3208, 0x530C,   // 0x00B0 (176) pixels
0x4B0C, 0x3208, 0x2145, 0x0841, 0x0000, 0x0000, 0x21A7, 0x3310, 0x23B6, 0x6CF9, 0xD6BA, 0xD6BA, 0xD6BA, 0xD69A, 0xD69A, 0xAD75,   // 0x00C0 (192) pixels
0xCE79, 0xD69A, 0xD69A, 0xD6BA, 0xCE7A, 0x5457, 0x2A2B, 0x0000, 0x0000, 0x1082, 0x428B, 0x84F5, 0xAE5A, 0xD6FC, 0xDF3D, 0xEF7D,   // 0x00D0 (208) pixels
0xEF7D, 0xE73D, 0xD6FC, 0xB65A, 0x8515, 0x4AEC, 0x0020, 0x0840, 0x0882, 0x2A8D, 0x3BB7, 0xC65A, 0xD6BA, 0xD69A, 0xD69A, 0xCE79,   // 0x00E0 (224) pixels
0xCE59, 0xD69A, 0xD69A, 0xB61A, 0x3355, 0x42AC, 0x0861, 0x1081, 0x534E, 0xAE19, 0xDEFB, 0xE71C, 0xE73C, 0xEF5D, 0xEF7D, 0xEF7D,   // 0x00F0 (240) pixels
0xEF7D, 0xEF5D, 0xEF5D, 0xE73C, 0xE71C, 0xDEFB, 0xBE5A, 0x63B0, 0x0841, 0x0861, 0x21CA, 0x2314, 0xA5B9, 0xD69A, 0xD69A, 0xCE79,   // 0x0100 (256) pixels
0x5331, 0xA598, 0x9538, 0x2AF3, 0x29A8, 0x0841, 0x2986, 0x9557, 0xD6BA, 0xDEFB, 0xE71C, 0xE71C, 0xE73C, 0xEF5D, 0xEF5D, 0xEF7D,   // 0x0110 (272) pixels
0xEF7D, 0xEF7D, 0xEF5D, 0xE73C, 0xE71C, 0xDEFB, 0xDEFB, 0xD6DB, 0xA5B8, 0x2986, 0x0020, 0x3A6C, 0x2B15, 0x84D8, 0xAD98, 0x6BB2,   // 0x0120 (288) pixels
0x10A3, 0x1967, 0x21CA, 0x1082, 0x0000, 0x3A4A, 0xADF9, 0xD6BA, 0xDEDB, 0xDEFB, 0xE71C, 0xE71C, 0xD6FC, 0xB69B, 0x967B, 0x7E3B,   // 0x0130 (304) pixels
0x7E3B, 0x863B, 0xAE7B, 0xCEDB, 0xE71C, 0xDEFB, 0xDEFB, 0xDEDB, 0xD6BA, 0xBE39, 0x636E, 0x0000, 0x10A3, 0x1969, 0x1927, 0x0882,   // 0x0140 (320) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x2986, 0xB619, 0xD69A, 0xD6BA, 0xDEDB, 0xDEFB, 0xBE7B, 0x7DBA, 0x44D7, 0x2B91, 0x22EE, 0x228C,   // 0x0150 (336) pixels
0x22AC, 0x230E, 0x2BD2, 0x3CB7, 0x6D79, 0xAE5A, 0xDEDB, 0xDEDB, 0xD6BA, 0xD69A, 0xC659, 0x31C7, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0160 (352) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x63B0, 0xD69A, 0xD69A, 0xD6BA, 0xD6BB, 0x8DDA, 0x3435, 0x220A, 0x10C3, 0x0000, 0x0000, 0x0000,   // 0x0170 (368) pixels
0x0000, 0x0000, 0x0000, 0x1925, 0x2AAD, 0x3477, 0x757A, 0xCE9A, 0xD6BA, 0xD69A, 0xCE79, 0x73F0, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0180 (384) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x3A8C, 0xBE39, 0xD69A, 0xBE5A, 0x7D79, 0x2A09, 0x0000, 0x0020, 0x31E7, 0x43B0, 0x3C75, 0x3D18,   // 0x0190 (400) pixels
0x3D39, 0x3C95, 0x43B0, 0x2186, 0x0020, 0x0841, 0x32CD, 0x54D8, 0xAE19, 0xD69A, 0xC659, 0x52ED, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01A0 (416) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x18E4, 0x53F4, 0x74F7, 0x5455, 0x21C8, 0x0020, 0x0861, 0x5433, 0x3519, 0x0D3B, 0x05BC, 0x05FC,   // 0x01B0 (432) pixels
0x061C, 0x05BC, 0x0D5B, 0x24F9, 0x4C33, 0x18E3, 0x0841, 0x2A2B, 0x4C56, 0x6496, 0x5C13, 0x0862, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01C0 (448) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x18E4, 0x1967, 0x18E4, 0x0000, 0x2986, 0x4D18, 0x14BA, 0x053B, 0x057B, 0x05BC, 0x05FC,   // 0x01D0 (464) pixels
0x05FC, 0x05BC, 0x057B, 0x053B, 0x0CBA, 0x44B7, 0x2145, 0x0041, 0x2166, 0x1967, 0x18C3, 0x0841, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x01E0 (480) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4C76, 0x047A, 0x04DA, 0x051B, 0x055B, 0x05BC, 0x0DDC,   // 0x01F0 (496) pixels
0x0DDC, 0x05BC, 0x057B, 0x053B, 0x04DA, 0x049A, 0x4497, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0200 (512) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3C98, 0x049A, 0x04DA, 0x0D1B, 0x355A, 0x5599, 0x4518,   // 0x0210 (528) pixels
0x4518, 0x559A, 0x3D7A, 0x0D1A, 0x04DB, 0x049A, 0x2458, 0x1082, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0220 (544) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x64F7, 0x1C79, 0x0CBA, 0x4D59, 0x4453, 0x3A8B, 0x2145,   // 0x0230 (560) pixels
0x2165, 0x3A8B, 0x54F6, 0x5579, 0x1CB9, 0x0C79, 0x64F7, 0x0020, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0240 (576) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x2186, 0x5C95, 0x4C74, 0x324A, 0x0000, 0x0020, 0x426A,   // 0x0250 (592) pixels
0x29A7, 0x0861, 0x0020, 0x3ACC, 0x4C54, 0x5C74, 0x3A08, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0260 (608) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x10A3, 0x0882, 0x0000, 0x10A2, 0x7519, 0x1BB7,   // 0x0270 (624) pixels
0x0B97, 0x4C15, 0x0861, 0x0041, 0x0882, 0x10A2, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0280 (640) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5370, 0x0BD8, 0x053B,   // 0x0290 (656) pixels
0x053B, 0x03F8, 0x4B70, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x02A0 (672) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x4392, 0x0419, 0x057C,   // 0x02B0 (688) pixels
0x057C, 0x041A, 0x4392, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x02C0 (704) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5370, 0x23B7, 0x045A,   // 0x02D0 (720) pixels
0x045A, 0x1397, 0x42ED, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x02E0 (736) pixels
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x18E3, 0x5456, 0x2BD6,   // 0x02F0 (752) pixels
0x2396, 0x5436, 0x2145, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,   // 0x0300 (768) pixels
};