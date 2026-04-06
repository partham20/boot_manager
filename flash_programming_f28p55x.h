//#############################################################################
//
// FILE:   flash_programming_f28p55x.h
//
// TITLE:  A set of Constant Values for the F28P55x Family.
//
//#############################################################################
//
//
// 
// C2000Ware v5.04.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef FLASH_PROGRAMMING_F28P55X_H_
#define FLASH_PROGRAMMING_F28P55X_H_

// Bank0 Sector start addresses
#define FlashBank0StartAddress      0x80000U
#define Bzero_Sector0_start         0x80000U
#define Bzero_Sector1_start         0x80400U
#define Bzero_Sector2_start         0x80800U
#define Bzero_Sector3_start         0x80C00U
#define Bzero_Sector4_start         0x81000U
#define Bzero_Sector5_start         0x81400U
#define Bzero_Sector6_start         0x81800U
#define Bzero_Sector7_start         0x81C00U
#define Bzero_Sector8_start         0x82000U
#define Bzero_Sector9_start         0x82400U
#define Bzero_Sector10_start        0x82800U
#define Bzero_Sector11_start        0x82C00U
#define Bzero_Sector12_start        0x83000U
#define Bzero_Sector13_start        0x83400U
#define Bzero_Sector14_start        0x83800U
#define Bzero_Sector15_start        0x83C00U
#define Bzero_Sector16_start        0x84000U
#define Bzero_Sector17_start        0x84400U
#define Bzero_Sector18_start        0x84800U
#define Bzero_Sector19_start        0x84C00U
#define Bzero_Sector20_start        0x85000U
#define Bzero_Sector21_start        0x85400U
#define Bzero_Sector22_start        0x85800U
#define Bzero_Sector23_start        0x85C00U
#define Bzero_Sector24_start        0x86000U
#define Bzero_Sector25_start        0x86400U
#define Bzero_Sector26_start        0x86800U
#define Bzero_Sector27_start        0x86C00U
#define Bzero_Sector28_start        0x87000U
#define Bzero_Sector29_start        0x87400U
#define Bzero_Sector30_start        0x87800U
#define Bzero_Sector31_start        0x87C00U
#define Bzero_Sector32_start        0x88000U
#define Bzero_Sector33_start        0x88400U
#define Bzero_Sector34_start        0x88800U
#define Bzero_Sector35_start        0x88C00U
#define Bzero_Sector36_start        0x89000U
#define Bzero_Sector37_start        0x89400U
#define Bzero_Sector38_start        0x89800U
#define Bzero_Sector39_start        0x89C00U
#define Bzero_Sector40_start        0x8A000U
#define Bzero_Sector41_start        0x8A400U
#define Bzero_Sector42_start        0x8A800U
#define Bzero_Sector43_start        0x8AC00U
#define Bzero_Sector44_start        0x8B000U
#define Bzero_Sector45_start        0x8B400U
#define Bzero_Sector46_start        0x8B800U
#define Bzero_Sector47_start        0x8BC00U
#define Bzero_Sector48_start        0x8C000U
#define Bzero_Sector49_start        0x8C400U
#define Bzero_Sector50_start        0x8C800U
#define Bzero_Sector51_start        0x8CC00U
#define Bzero_Sector52_start        0x8D000U
#define Bzero_Sector53_start        0x8D400U
#define Bzero_Sector54_start        0x8D800U
#define Bzero_Sector55_start        0x8DC00U
#define Bzero_Sector56_start        0x8E000U
#define Bzero_Sector57_start        0x8E400U
#define Bzero_Sector58_start        0x8E800U
#define Bzero_Sector59_start        0x8EC00U
#define Bzero_Sector60_start        0x8F000U
#define Bzero_Sector61_start        0x8F400U
#define Bzero_Sector62_start        0x8F800U
#define Bzero_Sector63_start        0x8FC00U
#define Bzero_Sector64_start        0x90000U
#define Bzero_Sector65_start        0x90400U
#define Bzero_Sector66_start        0x90800U
#define Bzero_Sector67_start        0x90C00U
#define Bzero_Sector68_start        0x91000U
#define Bzero_Sector69_start        0x91400U
#define Bzero_Sector70_start        0x91800U
#define Bzero_Sector71_start        0x91C00U
#define Bzero_Sector72_start        0x92000U
#define Bzero_Sector73_start        0x92400U
#define Bzero_Sector74_start        0x92800U
#define Bzero_Sector75_start        0x92C00U
#define Bzero_Sector76_start        0x93000U
#define Bzero_Sector77_start        0x93400U
#define Bzero_Sector78_start        0x93800U
#define Bzero_Sector79_start        0x93C00U
#define Bzero_Sector80_start        0x94000U
#define Bzero_Sector81_start        0x94400U
#define Bzero_Sector82_start        0x94800U
#define Bzero_Sector83_start        0x94C00U
#define Bzero_Sector84_start        0x95000U
#define Bzero_Sector85_start        0x95400U
#define Bzero_Sector86_start        0x95800U
#define Bzero_Sector87_start        0x95C00U
#define Bzero_Sector88_start        0x96000U
#define Bzero_Sector89_start        0x96400U
#define Bzero_Sector90_start        0x96800U
#define Bzero_Sector91_start        0x96C00U
#define Bzero_Sector92_start        0x97000U
#define Bzero_Sector93_start        0x97400U
#define Bzero_Sector94_start        0x97800U
#define Bzero_Sector95_start        0x97C00U
#define Bzero_Sector96_start        0x98000U
#define Bzero_Sector97_start        0x98400U
#define Bzero_Sector98_start        0x98800U
#define Bzero_Sector99_start        0x98C00U
#define Bzero_Sector100_start       0x99000U
#define Bzero_Sector101_start       0x99400U
#define Bzero_Sector102_start       0x99800U
#define Bzero_Sector103_start       0x99C00U
#define Bzero_Sector104_start       0x9A000U
#define Bzero_Sector105_start       0x9A400U
#define Bzero_Sector106_start       0x9A800U
#define Bzero_Sector107_start       0x9AC00U
#define Bzero_Sector108_start       0x9B000U
#define Bzero_Sector109_start       0x9B400U
#define Bzero_Sector110_start       0x9B800U
#define Bzero_Sector111_start       0x9BC00U
#define Bzero_Sector112_start       0x9C000U
#define Bzero_Sector113_start       0x9C400U
#define Bzero_Sector114_start       0x9C800U
#define Bzero_Sector115_start       0x9CC00U
#define Bzero_Sector116_start       0x9D000U
#define Bzero_Sector117_start       0x9D400U
#define Bzero_Sector118_start       0x9D800U
#define Bzero_Sector119_start       0x9DC00U
#define Bzero_Sector120_start       0x9E000U
#define Bzero_Sector121_start       0x9E400U
#define Bzero_Sector122_start       0x9E800U
#define Bzero_Sector123_start       0x9EC00U
#define Bzero_Sector124_start       0x9F000U
#define Bzero_Sector125_start       0x9F400U
#define Bzero_Sector126_start       0x9F800U
#define Bzero_Sector127_start       0x9FC00U
#define FlashBank0EndAddress        0x9FFFFU

// Bank1 Sector start addresses
#define FlashBank1StartAddress     0xA0000U
#define Bone_Sector0_start         0xA0000U
#define Bone_Sector1_start         0xA0400U
#define Bone_Sector2_start         0xA0800U
#define Bone_Sector3_start         0xA0C00U
#define Bone_Sector4_start         0xA1000U
#define Bone_Sector5_start         0xA1400U
#define Bone_Sector6_start         0xA1800U
#define Bone_Sector7_start         0xA1C00U
#define Bone_Sector8_start         0xA2000U
#define Bone_Sector9_start         0xA2400U
#define Bone_Sector10_start        0xA2800U
#define Bone_Sector11_start        0xA2C00U
#define Bone_Sector12_start        0xA3000U
#define Bone_Sector13_start        0xA3400U
#define Bone_Sector14_start        0xA3800U
#define Bone_Sector15_start        0xA3C00U
#define Bone_Sector16_start        0xA4000U
#define Bone_Sector17_start        0xA4400U
#define Bone_Sector18_start        0xA4800U
#define Bone_Sector19_start        0xA4C00U
#define Bone_Sector20_start        0xA5000U
#define Bone_Sector21_start        0xA5400U
#define Bone_Sector22_start        0xA5800U
#define Bone_Sector23_start        0xA5C00U
#define Bone_Sector24_start        0xA6000U
#define Bone_Sector25_start        0xA6400U
#define Bone_Sector26_start        0xA6800U
#define Bone_Sector27_start        0xA6C00U
#define Bone_Sector28_start        0xA7000U
#define Bone_Sector29_start        0xA7400U
#define Bone_Sector30_start        0xA7800U
#define Bone_Sector31_start        0xA7C00U
#define Bone_Sector32_start        0xA8000U
#define Bone_Sector33_start        0xA8400U
#define Bone_Sector34_start        0xA8800U
#define Bone_Sector35_start        0xA8C00U
#define Bone_Sector36_start        0xA9000U
#define Bone_Sector37_start        0xA9400U
#define Bone_Sector38_start        0xA9800U
#define Bone_Sector39_start        0xA9C00U
#define Bone_Sector40_start        0xAA000U
#define Bone_Sector41_start        0xAA400U
#define Bone_Sector42_start        0xAA800U
#define Bone_Sector43_start        0xAAC00U
#define Bone_Sector44_start        0xAB000U
#define Bone_Sector45_start        0xAB400U
#define Bone_Sector46_start        0xAB800U
#define Bone_Sector47_start        0xABC00U
#define Bone_Sector48_start        0xAC000U
#define Bone_Sector49_start        0xAC400U
#define Bone_Sector50_start        0xAC800U
#define Bone_Sector51_start        0xACC00U
#define Bone_Sector52_start        0xAD000U
#define Bone_Sector53_start        0xAD400U
#define Bone_Sector54_start        0xAD800U
#define Bone_Sector55_start        0xADC00U
#define Bone_Sector56_start        0xAE000U
#define Bone_Sector57_start        0xAE400U
#define Bone_Sector58_start        0xAE800U
#define Bone_Sector59_start        0xAEC00U
#define Bone_Sector60_start        0xAF000U
#define Bone_Sector61_start        0xAF400U
#define Bone_Sector62_start        0xAF800U
#define Bone_Sector63_start        0xAFC00U
#define Bone_Sector64_start        0xB0000U
#define Bone_Sector65_start        0xB0400U
#define Bone_Sector66_start        0xB0800U
#define Bone_Sector67_start        0xB0C00U
#define Bone_Sector68_start        0xB1000U
#define Bone_Sector69_start        0xB1400U
#define Bone_Sector70_start        0xB1800U
#define Bone_Sector71_start        0xB1C00U
#define Bone_Sector72_start        0xB2000U
#define Bone_Sector73_start        0xB2400U
#define Bone_Sector74_start        0xB2800U
#define Bone_Sector75_start        0xB2C00U
#define Bone_Sector76_start        0xB3000U
#define Bone_Sector77_start        0xB3400U
#define Bone_Sector78_start        0xB3800U
#define Bone_Sector79_start        0xB3C00U
#define Bone_Sector80_start        0xB4000U
#define Bone_Sector81_start        0xB4400U
#define Bone_Sector82_start        0xB4800U
#define Bone_Sector83_start        0xB4C00U
#define Bone_Sector84_start        0xB5000U
#define Bone_Sector85_start        0xB5400U
#define Bone_Sector86_start        0xB5800U
#define Bone_Sector87_start        0xB5C00U
#define Bone_Sector88_start        0xB6000U
#define Bone_Sector89_start        0xB6400U
#define Bone_Sector90_start        0xB6800U
#define Bone_Sector91_start        0xB6C00U
#define Bone_Sector92_start        0xB7000U
#define Bone_Sector93_start        0xB7400U
#define Bone_Sector94_start        0xB7800U
#define Bone_Sector95_start        0xB7C00U
#define Bone_Sector96_start        0xB8000U
#define Bone_Sector97_start        0xB8400U
#define Bone_Sector98_start        0xB8800U
#define Bone_Sector99_start        0xB8C00U
#define Bone_Sector100_start       0xB9000U
#define Bone_Sector101_start       0xB9400U
#define Bone_Sector102_start       0xB9800U
#define Bone_Sector103_start       0xB9C00U
#define Bone_Sector104_start       0xBA000U
#define Bone_Sector105_start       0xBA400U
#define Bone_Sector106_start       0xBA800U
#define Bone_Sector107_start       0xBAC00U
#define Bone_Sector108_start       0xBB000U
#define Bone_Sector109_start       0xBB400U
#define Bone_Sector110_start       0xBB800U
#define Bone_Sector111_start       0xBBC00U
#define Bone_Sector112_start       0xBC000U
#define Bone_Sector113_start       0xBC400U
#define Bone_Sector114_start       0xBC800U
#define Bone_Sector115_start       0xBCC00U
#define Bone_Sector116_start       0xBD000U
#define Bone_Sector117_start       0xBD400U
#define Bone_Sector118_start       0xBD800U
#define Bone_Sector119_start       0xBDC00U
#define Bone_Sector120_start       0xBE000U
#define Bone_Sector121_start       0xBE400U
#define Bone_Sector122_start       0xBE800U
#define Bone_Sector123_start       0xBEC00U
#define Bone_Sector124_start       0xBF000U
#define Bone_Sector125_start       0xBF400U
#define Bone_Sector126_start       0xBF800U
#define Bone_Sector127_start       0xBFC00U
#define FlashBank1EndAddress       0xBFFFFU

// Bank2 Sector start addresses
#define FlashBank2StartAddress     0xC0000U
#define Btwo_Sector0_start         0xC0000U
#define Btwo_Sector1_start         0xC0400U
#define Btwo_Sector2_start         0xC0800U
#define Btwo_Sector3_start         0xC0C00U
#define Btwo_Sector4_start         0xC1000U
#define Btwo_Sector5_start         0xC1400U
#define Btwo_Sector6_start         0xC1800U
#define Btwo_Sector7_start         0xC1C00U
#define Btwo_Sector8_start         0xC2000U
#define Btwo_Sector9_start         0xC2400U
#define Btwo_Sector10_start        0xC2800U
#define Btwo_Sector11_start        0xC2C00U
#define Btwo_Sector12_start        0xC3000U
#define Btwo_Sector13_start        0xC3400U
#define Btwo_Sector14_start        0xC3800U
#define Btwo_Sector15_start        0xC3C00U
#define Btwo_Sector16_start        0xC4000U
#define Btwo_Sector17_start        0xC4400U
#define Btwo_Sector18_start        0xC4800U
#define Btwo_Sector19_start        0xC4C00U
#define Btwo_Sector20_start        0xC5000U
#define Btwo_Sector21_start        0xC5400U
#define Btwo_Sector22_start        0xC5800U
#define Btwo_Sector23_start        0xC5C00U
#define Btwo_Sector24_start        0xC6000U
#define Btwo_Sector25_start        0xC6400U
#define Btwo_Sector26_start        0xC6800U
#define Btwo_Sector27_start        0xC6C00U
#define Btwo_Sector28_start        0xC7000U
#define Btwo_Sector29_start        0xC7400U
#define Btwo_Sector30_start        0xC7800U
#define Btwo_Sector31_start        0xC7C00U
#define Btwo_Sector32_start        0xC8000U
#define Btwo_Sector33_start        0xC8400U
#define Btwo_Sector34_start        0xC8800U
#define Btwo_Sector35_start        0xC8C00U
#define Btwo_Sector36_start        0xC9000U
#define Btwo_Sector37_start        0xC9400U
#define Btwo_Sector38_start        0xC9800U
#define Btwo_Sector39_start        0xC9C00U
#define Btwo_Sector40_start        0xCA000U
#define Btwo_Sector41_start        0xCA400U
#define Btwo_Sector42_start        0xCA800U
#define Btwo_Sector43_start        0xCAC00U
#define Btwo_Sector44_start        0xCB000U
#define Btwo_Sector45_start        0xCB400U
#define Btwo_Sector46_start        0xCB800U
#define Btwo_Sector47_start        0xCBC00U
#define Btwo_Sector48_start        0xCC000U
#define Btwo_Sector49_start        0xCC400U
#define Btwo_Sector50_start        0xCC800U
#define Btwo_Sector51_start        0xCCC00U
#define Btwo_Sector52_start        0xCD000U
#define Btwo_Sector53_start        0xCD400U
#define Btwo_Sector54_start        0xCD800U
#define Btwo_Sector55_start        0xCDC00U
#define Btwo_Sector56_start        0xCE000U
#define Btwo_Sector57_start        0xCE400U
#define Btwo_Sector58_start        0xCE800U
#define Btwo_Sector59_start        0xCEC00U
#define Btwo_Sector60_start        0xCF000U
#define Btwo_Sector61_start        0xCF400U
#define Btwo_Sector62_start        0xCF800U
#define Btwo_Sector63_start        0xCFC00U
#define Btwo_Sector64_start        0xD0000U
#define Btwo_Sector65_start        0xD0400U
#define Btwo_Sector66_start        0xD0800U
#define Btwo_Sector67_start        0xD0C00U
#define Btwo_Sector68_start        0xD1000U
#define Btwo_Sector69_start        0xD1400U
#define Btwo_Sector70_start        0xD1800U
#define Btwo_Sector71_start        0xD1C00U
#define Btwo_Sector72_start        0xD2000U
#define Btwo_Sector73_start        0xD2400U
#define Btwo_Sector74_start        0xD2800U
#define Btwo_Sector75_start        0xD2C00U
#define Btwo_Sector76_start        0xD3000U
#define Btwo_Sector77_start        0xD3400U
#define Btwo_Sector78_start        0xD3800U
#define Btwo_Sector79_start        0xD3C00U
#define Btwo_Sector80_start        0xD4000U
#define Btwo_Sector81_start        0xD4400U
#define Btwo_Sector82_start        0xD4800U
#define Btwo_Sector83_start        0xD4C00U
#define Btwo_Sector84_start        0xD5000U
#define Btwo_Sector85_start        0xD5400U
#define Btwo_Sector86_start        0xD5800U
#define Btwo_Sector87_start        0xD5C00U
#define Btwo_Sector88_start        0xD6000U
#define Btwo_Sector89_start        0xD6400U
#define Btwo_Sector90_start        0xD6800U
#define Btwo_Sector91_start        0xD6C00U
#define Btwo_Sector92_start        0xD7000U
#define Btwo_Sector93_start        0xD7400U
#define Btwo_Sector94_start        0xD7800U
#define Btwo_Sector95_start        0xD7C00U
#define Btwo_Sector96_start        0xD8000U
#define Btwo_Sector97_start        0xD8400U
#define Btwo_Sector98_start        0xD8800U
#define Btwo_Sector99_start        0xD8C00U
#define Btwo_Sector100_start       0xD9000U
#define Btwo_Sector101_start       0xD9400U
#define Btwo_Sector102_start       0xD9800U
#define Btwo_Sector103_start       0xD9C00U
#define Btwo_Sector104_start       0xDA000U
#define Btwo_Sector105_start       0xDA400U
#define Btwo_Sector106_start       0xDA800U
#define Btwo_Sector107_start       0xDAC00U
#define Btwo_Sector108_start       0xDB000U
#define Btwo_Sector109_start       0xDB400U
#define Btwo_Sector110_start       0xDB800U
#define Btwo_Sector111_start       0xDBC00U
#define Btwo_Sector112_start       0xDC000U
#define Btwo_Sector113_start       0xDC400U
#define Btwo_Sector114_start       0xDC800U
#define Btwo_Sector115_start       0xDCC00U
#define Btwo_Sector116_start       0xDD000U
#define Btwo_Sector117_start       0xDD400U
#define Btwo_Sector118_start       0xDD800U
#define Btwo_Sector119_start       0xDDC00U
#define Btwo_Sector120_start       0xDE000U
#define Btwo_Sector121_start       0xDE400U
#define Btwo_Sector122_start       0xDE800U
#define Btwo_Sector123_start       0xDEC00U
#define Btwo_Sector124_start       0xDF000U
#define Btwo_Sector125_start       0xDF400U
#define Btwo_Sector126_start       0xDF800U
#define Btwo_Sector127_start       0xDFC00U
#define FlashBank2EndAddress       0xDFFFFU

// Bank3 Sector start addresses
#define FlashBank3StartAddress       0xE0000U
#define Bthree_Sector0_start         0xE0000U
#define Bthree_Sector1_start         0xE0400U
#define Bthree_Sector2_start         0xE0800U
#define Bthree_Sector3_start         0xE0C00U
#define Bthree_Sector4_start         0xE1000U
#define Bthree_Sector5_start         0xE1400U
#define Bthree_Sector6_start         0xE1800U
#define Bthree_Sector7_start         0xE1C00U
#define Bthree_Sector8_start         0xE2000U
#define Bthree_Sector9_start         0xE2400U
#define Bthree_Sector10_start        0xE2800U
#define Bthree_Sector11_start        0xE2C00U
#define Bthree_Sector12_start        0xE3000U
#define Bthree_Sector13_start        0xE3400U
#define Bthree_Sector14_start        0xE3800U
#define Bthree_Sector15_start        0xE3C00U
#define Bthree_Sector16_start        0xE4000U
#define Bthree_Sector17_start        0xE4400U
#define Bthree_Sector18_start        0xE4800U
#define Bthree_Sector19_start        0xE4C00U
#define Bthree_Sector20_start        0xE5000U
#define Bthree_Sector21_start        0xE5400U
#define Bthree_Sector22_start        0xE5800U
#define Bthree_Sector23_start        0xE5C00U
#define Bthree_Sector24_start        0xE6000U
#define Bthree_Sector25_start        0xE6400U
#define Bthree_Sector26_start        0xE6800U
#define Bthree_Sector27_start        0xE6C00U
#define Bthree_Sector28_start        0xE7000U
#define Bthree_Sector29_start        0xE7400U
#define Bthree_Sector30_start        0xE7800U
#define Bthree_Sector31_start        0xE7C00U
#define Bthree_Sector32_start        0xE8000U
#define Bthree_Sector33_start        0xE8400U
#define Bthree_Sector34_start        0xE8800U
#define Bthree_Sector35_start        0xE8C00U
#define Bthree_Sector36_start        0xE9000U
#define Bthree_Sector37_start        0xE9400U
#define Bthree_Sector38_start        0xE9800U
#define Bthree_Sector39_start        0xE9C00U
#define Bthree_Sector40_start        0xEA000U
#define Bthree_Sector41_start        0xEA400U
#define Bthree_Sector42_start        0xEA800U
#define Bthree_Sector43_start        0xEAC00U
#define Bthree_Sector44_start        0xEB000U
#define Bthree_Sector45_start        0xEB400U
#define Bthree_Sector46_start        0xEB800U
#define Bthree_Sector47_start        0xEBC00U
#define Bthree_Sector48_start        0xEC000U
#define Bthree_Sector49_start        0xEC400U
#define Bthree_Sector50_start        0xEC800U
#define Bthree_Sector51_start        0xECC00U
#define Bthree_Sector52_start        0xED000U
#define Bthree_Sector53_start        0xED400U
#define Bthree_Sector54_start        0xED800U
#define Bthree_Sector55_start        0xEDC00U
#define Bthree_Sector56_start        0xEE000U
#define Bthree_Sector57_start        0xEE400U
#define Bthree_Sector58_start        0xEE800U
#define Bthree_Sector59_start        0xEEC00U
#define Bthree_Sector60_start        0xEF000U
#define Bthree_Sector61_start        0xEF400U
#define Bthree_Sector62_start        0xEF800U
#define Bthree_Sector63_start        0xEFC00U
#define Bthree_Sector64_start        0xF0000U
#define Bthree_Sector65_start        0xF0400U
#define Bthree_Sector66_start        0xF0800U
#define Bthree_Sector67_start        0xF0C00U
#define Bthree_Sector68_start        0xF1000U
#define Bthree_Sector69_start        0xF1400U
#define Bthree_Sector70_start        0xF1800U
#define Bthree_Sector71_start        0xF1C00U
#define Bthree_Sector72_start        0xF2000U
#define Bthree_Sector73_start        0xF2400U
#define Bthree_Sector74_start        0xF2800U
#define Bthree_Sector75_start        0xF2C00U
#define Bthree_Sector76_start        0xF3000U
#define Bthree_Sector77_start        0xF3400U
#define Bthree_Sector78_start        0xF3800U
#define Bthree_Sector79_start        0xF3C00U
#define Bthree_Sector80_start        0xF4000U
#define Bthree_Sector81_start        0xF4400U
#define Bthree_Sector82_start        0xF4800U
#define Bthree_Sector83_start        0xF4C00U
#define Bthree_Sector84_start        0xF5000U
#define Bthree_Sector85_start        0xF5400U
#define Bthree_Sector86_start        0xF5800U
#define Bthree_Sector87_start        0xF5C00U
#define Bthree_Sector88_start        0xF6000U
#define Bthree_Sector89_start        0xF6400U
#define Bthree_Sector90_start        0xF6800U
#define Bthree_Sector91_start        0xF6C00U
#define Bthree_Sector92_start        0xF7000U
#define Bthree_Sector93_start        0xF7400U
#define Bthree_Sector94_start        0xF7800U
#define Bthree_Sector95_start        0xF7C00U
#define Bthree_Sector96_start        0xF8000U
#define Bthree_Sector97_start        0xF8400U
#define Bthree_Sector98_start        0xF8800U
#define Bthree_Sector99_start        0xF8C00U
#define Bthree_Sector100_start       0xF9000U
#define Bthree_Sector101_start       0xF9400U
#define Bthree_Sector102_start       0xF9800U
#define Bthree_Sector103_start       0xF9C00U
#define Bthree_Sector104_start       0xFA000U
#define Bthree_Sector105_start       0xFA400U
#define Bthree_Sector106_start       0xFA800U
#define Bthree_Sector107_start       0xFAC00U
#define Bthree_Sector108_start       0xFB000U
#define Bthree_Sector109_start       0xFB400U
#define Bthree_Sector110_start       0xFB800U
#define Bthree_Sector111_start       0xFBC00U
#define Bthree_Sector112_start       0xFC000U
#define Bthree_Sector113_start       0xFC400U
#define Bthree_Sector114_start       0xFC800U
#define Bthree_Sector115_start       0xFCC00U
#define Bthree_Sector116_start       0xFD000U
#define Bthree_Sector117_start       0xFD400U
#define Bthree_Sector118_start       0xFD800U
#define Bthree_Sector119_start       0xFDC00U
#define Bthree_Sector120_start       0xFE000U
#define Bthree_Sector121_start       0xFE400U
#define Bthree_Sector122_start       0xFE800U
#define Bthree_Sector123_start       0xFEC00U
#define Bthree_Sector124_start       0xFF000U
#define Bthree_Sector125_start       0xFF400U
#define Bthree_Sector126_start       0xFF800U
#define Bthree_Sector127_start       0xFFC00U
#define FlashBank3EndAddress         0xFFFFFU

// Bank4 Sector start addresses
#define FlashBank4StartAddress      0x100000U
#define Bfour_Sector0_start         0x100000U
#define Bfour_Sector1_start         0x100400U
#define Bfour_Sector2_start         0x100800U
#define Bfour_Sector3_start         0x100C00U
#define Bfour_Sector4_start         0x101000U
#define Bfour_Sector5_start         0x101400U
#define Bfour_Sector6_start         0x101800U
#define Bfour_Sector7_start         0x101C00U
#define Bfour_Sector8_start         0x102000U
#define Bfour_Sector9_start         0x102400U
#define Bfour_Sector10_start        0x102800U
#define Bfour_Sector11_start        0x102C00U
#define Bfour_Sector12_start        0x103000U
#define Bfour_Sector13_start        0x103400U
#define Bfour_Sector14_start        0x103800U
#define Bfour_Sector15_start        0x103C00U
#define Bfour_Sector16_start        0x104000U
#define Bfour_Sector17_start        0x104400U
#define Bfour_Sector18_start        0x104800U
#define Bfour_Sector19_start        0x104C00U
#define Bfour_Sector20_start        0x105000U
#define Bfour_Sector21_start        0x105400U
#define Bfour_Sector22_start        0x105800U
#define Bfour_Sector23_start        0x105C00U
#define Bfour_Sector24_start        0x106000U
#define Bfour_Sector25_start        0x106400U
#define Bfour_Sector26_start        0x106800U
#define Bfour_Sector27_start        0x106C00U
#define Bfour_Sector28_start        0x107000U
#define Bfour_Sector29_start        0x107400U
#define Bfour_Sector30_start        0x107800U
#define Bfour_Sector31_start        0x107C00U
#define FlashBank4EndAddress        0x107FFFU

//Bank offset
#define     FlashBankOffset        0x20000U

//Sector length in number of 16bits
#define Sector2KB_u16length   0x400U
#define Bank256KB_u16length   0x20000U

//Sector length in number of 32bits
#define Sector2KB_u32length   0x200U
#define Bank256KB_u32length   0x10000U

//Flash wrapper base address
#define FLASH_WRAPPER_PROGRAM_BASE         0x51000U /*(FLASH0CMD_BASE - 0x1000UL)*/
#define FlashTech_CPU0_BASE_ADDRESS ((Fapi_FmcRegistersType *)FLASH_WRAPPER_PROGRAM_BASE)

//
// FMC Registers, Masks and Values
//
// #define FLASH_O_ACC_EP           0x148U    // Flash Accumulated Erase Pulses
// #define FLASH_O_ERA_PUL          0x136U    // Flash Max Erase Pulses
// #define FLASH_O_OPT              0x144U    // Flash Optimization

// #define FLASH_ERA_PUL_MAX_ERA_PUL_M      0xFFFU  // Flash Max Erase Pulses Mask
// #define OPT_ENABLE                       0x5U    // Flash Optimization Enable
// #define OPT_DISABLE                      0x2U    // Flash Optimization Disable
// #define MAX_ERASE_PULSE                  0x7D0U  // Flash Max Erase Pulses

#endif /* FLASH_PROGRAMMING_F28P55X_H_ */
