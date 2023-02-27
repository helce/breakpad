/* Copyright (c) 2013, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *     * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/* minidump_format.h: A cross-platform reimplementation of minidump-related
 * portions of DbgHelp.h from the Windows Platform SDK.
 *
 * (This is C99 source, please don't corrupt it with C++.)
 *
 * This file contains the necessary definitions to read minidump files
 * produced on E2K.  These files may be read on any platform provided
 * that the alignments of these structures on the processing system are
 * identical to the alignments of these structures on the producing system.
 * For this reason, precise-sized types are used.  The structures defined
 * by this file have been laid out to minimize alignment problems by
 * ensuring that all members are aligned on their natural boundaries.
 * In some cases, tail-padding may be significant when different ABIs specify
 * different tail-padding behaviors.  To avoid problems when reading or
 * writing affected structures, MD_*_SIZE macros are provided where needed,
 * containing the useful size of the structures without padding.
 *
 * Structures that are defined by Microsoft to contain a zero-length array
 * are instead defined here to contain an array with one element, as
 * zero-length arrays are forbidden by standard C and C++.  In these cases,
 * *_minsize constants are provided to be used in place of sizeof.  For a
 * cleaner interface to these sizes when using C++, see minidump_size.h.
 *
 * These structures are also sufficient to populate minidump files.
 *
 * Because precise data type sizes are crucial for this implementation to
 * function properly and portably, a set of primitive types with known sizes
 * are used as the basis of each structure defined by this file.
 *
 * Author: Svyatoslav Stupak
 */

/*
 * E2K support
 */

#ifndef GOOGLE_BREAKPAD_COMMON_MINIDUMP_CPU_E2K_H__
#define GOOGLE_BREAKPAD_COMMON_MINIDUMP_CPU_E2K_H__

#define MD_CONTEXT_E2K_GREGS_COUNT       32

// It is small part of all registers
typedef struct {
  uint32_t    context_flags;
  uint64_t    g[MD_CONTEXT_E2K_GREGS_COUNT];
  uint64_t    usbr;
  uint64_t    usd_lo;
  uint64_t    usd_hi;
  uint64_t    psp_lo;
  uint64_t    psp_hi;
  uint64_t    pshtp;
  uint64_t    cr0_lo;
  uint64_t    cr0_hi;
  uint64_t    cr1_lo;
  uint64_t    cr1_hi;
  uint64_t    pcsp_lo;
  uint64_t    pcsp_hi;
  uint64_t    pcshtp;
  uint64_t    ctpr1;
  uint64_t    ctpr2;
  uint64_t    ctpr3;
  uint64_t    ps;
  uint64_t    pcs;
} MDRawContextE2K; /* CONTEXT */

/* This value was chosen to avoid likely conflicts with MD_CONTEXT_*
 * for other CPUs. */
#define MD_CONTEXT_E2K                   0x00000800
#define MD_CONTEXT_E2K_FULL              MD_CONTEXT_E2K
#define MD_CONTEXT_E2K_ALL               MD_CONTEXT_E2K

#endif /* GOOGLE_BREAKPAD_COMMON_MINIDUMP_CPU_E2K_H__ */