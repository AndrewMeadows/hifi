//
//  CPUIdent.cpp
//
//  Created by Ryan Huffman on 3/25/16.
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "CPUIdent.h"

#include <QtCore/QtGlobal>
#include <string.h>

#include "CPUDetect.h"
void getCPUID(uint32_t* p, uint32_t eax) {
    cpuidex((int*) p, (int) eax, 0);
}
void getCPUIDEX(uint32_t* p, uint32_t eax, uint32_t ecx) {
    cpuidex((int*) p, (int) eax, (int) ecx);
}

const CPUIdent::CPUIdent_Internal CPUIdent::CPU_Rep;

std::vector<CPUIdent::Feature> CPUIdent::getAllFeatures() {
    std::vector<CPUIdent::Feature> features;

    features.push_back({ "3DNOW", CPUIdent::_3DNOW() });
    features.push_back({ "3DNOWEXT", CPUIdent::_3DNOWEXT() });
    features.push_back({ "ABM", CPUIdent::ABM() });
    features.push_back({ "ADX", CPUIdent::ADX() });
    features.push_back({ "AES", CPUIdent::AES() });
    features.push_back({ "AVX", CPUIdent::AVX() });
    features.push_back({ "AVX2", CPUIdent::AVX2() });
    features.push_back({ "AVX512CD", CPUIdent::AVX512CD() });
    features.push_back({ "AVX512ER", CPUIdent::AVX512ER() });
    features.push_back({ "AVX512F", CPUIdent::AVX512F() });
    features.push_back({ "AVX512PF", CPUIdent::AVX512PF() });
    features.push_back({ "BMI1", CPUIdent::BMI1() });
    features.push_back({ "BMI2", CPUIdent::BMI2() });
    features.push_back({ "CLFSH", CPUIdent::CLFSH() });
    features.push_back({ "CMPXCHG16B", CPUIdent::CMPXCHG16B() });
    features.push_back({ "CX8", CPUIdent::CX8() });
    features.push_back({ "ERMS", CPUIdent::ERMS() });
    features.push_back({ "F16C", CPUIdent::F16C() });
    features.push_back({ "FMA", CPUIdent::FMA() });
    features.push_back({ "FSGSBASE", CPUIdent::FSGSBASE() });
    features.push_back({ "FXSR", CPUIdent::FXSR() });
    features.push_back({ "HLE", CPUIdent::HLE() });
    features.push_back({ "INVPCID", CPUIdent::INVPCID() });
    features.push_back({ "LAHF", CPUIdent::LAHF() });
    features.push_back({ "LZCNT", CPUIdent::LZCNT() });
    features.push_back({ "MMX", CPUIdent::MMX() });
    features.push_back({ "MMXEXT", CPUIdent::MMXEXT() });
    features.push_back({ "MONITOR", CPUIdent::MONITOR() });
    features.push_back({ "MOVBE", CPUIdent::MOVBE() });
    features.push_back({ "MSR", CPUIdent::MSR() });
    features.push_back({ "OSXSAVE", CPUIdent::OSXSAVE() });
    features.push_back({ "PCLMULQDQ", CPUIdent::PCLMULQDQ() });
    features.push_back({ "POPCNT", CPUIdent::POPCNT() });
    features.push_back({ "PREFETCHWT1", CPUIdent::PREFETCHWT1() });
    features.push_back({ "RDRAND", CPUIdent::RDRAND() });
    features.push_back({ "RDSEED", CPUIdent::RDSEED() });
    features.push_back({ "RDTSCP", CPUIdent::RDTSCP() });
    features.push_back({ "RTM", CPUIdent::RTM() });
    features.push_back({ "SEP", CPUIdent::SEP() });
    features.push_back({ "SHA", CPUIdent::SHA() });
    features.push_back({ "SSE", CPUIdent::SSE() });
    features.push_back({ "SSE2", CPUIdent::SSE2() });
    features.push_back({ "SSE3", CPUIdent::SSE3() });
    features.push_back({ "SSE4.1", CPUIdent::SSE41() });
    features.push_back({ "SSE4.2", CPUIdent::SSE42() });
    features.push_back({ "SSE4a", CPUIdent::SSE4a() });
    features.push_back({ "SSSE3", CPUIdent::SSSE3() });
    features.push_back({ "SYSCALL", CPUIdent::SYSCALL() });
    features.push_back({ "TBM", CPUIdent::TBM() });
    features.push_back({ "XOP", CPUIdent::XOP() });
    features.push_back({ "XSAVE", CPUIdent::XSAVE() });

    return features;
};


CPUIdent::CPUIdent_Internal::CPUIdent_Internal()
        : nIds_{ 0 },
        nExIds_{ 0 },
        isIntel_{ false },
        isAMD_{ false },
        f_1_ECX_{ 0 },
        f_1_EDX_{ 0 },
        f_7_EBX_{ 0 },
        f_7_ECX_{ 0 },
        f_81_ECX_{ 0 },
        f_81_EDX_{ 0 },
        data_{},
        extdata_{}
    {
    //int cpuInfo[4] = {-1};
    std::array<uint32_t, 4> cpui;

    // Calling __cpuid with 0x0 as the function_id argument
    // gets the number of the highest valid function ID.
    getCPUID(cpui.data(), 0);
    nIds_ = cpui[0];

    for (uint32_t i = 0; i <= nIds_; ++i) {
        getCPUIDEX(cpui.data(), i, 0);
        data_.push_back(cpui);
    }

    // Capture vendor string
    char vendor[0x20];
    memset(vendor, 0, sizeof(vendor));
    *reinterpret_cast<int*>(vendor) = data_[0][1];
    *reinterpret_cast<int*>(vendor + 4) = data_[0][3];
    *reinterpret_cast<int*>(vendor + 8) = data_[0][2];
    vendor_ = vendor;
    if (vendor_ == "GenuineIntel") {
        isIntel_ = true;
    }
    else if (vendor_ == "AuthenticAMD") {
        isAMD_ = true;
    }

    // load bitset with flags for function 0x00000001
    if (nIds_ >= 1) {
        f_1_ECX_ = data_[1][2];
        f_1_EDX_ = data_[1][3];
    }

    // load bitset with flags for function 0x00000007
    if (nIds_ >= 7) {
        f_7_EBX_ = data_[7][1];
        f_7_ECX_ = data_[7][2];
    }

    // Calling __cpuid with 0x80000000 as the function_id argument
    // gets the number of the highest valid extended ID.
    getCPUID(cpui.data(), 0x80000000);
    nExIds_ = cpui[0];

    char brand[0x40];
    memset(brand, 0, sizeof(brand));

    for (uint32_t i = 0x80000000; i <= nExIds_; ++i) {
        getCPUIDEX(cpui.data(), i, 0);
        extdata_.push_back(cpui);
    }

    // load bitset with flags for function 0x80000001
    if (nExIds_ >= 0x80000001) {
        f_81_ECX_ = extdata_[1][2];
        f_81_EDX_ = extdata_[1][3];
    }

    // Interpret CPU brand string if reported
    if (nExIds_ >= 0x80000004) {
        memcpy(brand, extdata_[2].data(), sizeof(cpui));
        memcpy(brand + 16, extdata_[3].data(), sizeof(cpui));
        memcpy(brand + 32, extdata_[4].data(), sizeof(cpui));
        brand_ = brand;
    }
}


