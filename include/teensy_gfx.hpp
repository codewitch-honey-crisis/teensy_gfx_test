//***************************************************
// https://github.com/kurte/teensy_gfx
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library
//
// Warning this is Kurt's updated version which allows it to work on different SPI busses.
//
// On Teensy 3.x allows you to use on only one valid hardware CS pin  which must
// be used for DC
//
// On Teensy 4.x including Micromod you are free to use any digital pin for
// CS and DC, but you might get a modest speed increase if hardware CS is
// used for the DC pin
//
/***************************************************
  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// <SoftEgg>

// Additional graphics routines by Tim Trzepacz, SoftEgg LLC added December 2015
//(And then accidentally deleted and rewritten March 2016. Oops!)
// Gradient support
//----------------
//		fillRectVGradient	- fills area with vertical gradient
//		fillRectHGradient	- fills area with horizontal gradient
//		fillScreenVGradient - fills screen with vertical gradient
// 	fillScreenHGradient - fills screen with horizontal gradient

// Additional Color Support
//------------------------
//		color565toRGB		- converts 565 format 16 bit color to
// RGB
//		color565toRGB14		- converts 16 bit 565 format color to
// 14 bit RGB (2 bits clear for math and sign)
//		RGB14tocolor565		- converts 14 bit RGB back to 16 bit
// 565 format color

// Low Memory Bitmap Support
//-------------------------
// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap

// String Pixel Length support
//---------------------------
//		strPixelLen			- gets pixel length of given ASCII
// string

// <\SoftEgg>
// Also some of this comes from the DMA version of the library...

/* ILI9341_t3DMA library code is placed under the MIT license
 * Copyright (c) 2016 Frank Bösing
 *
 */

#ifndef _ILI9341_t3NH_
#define _ILI9341_t3NH_
#define DISABLE_ILI9341_FRAMEBUFFER
#ifdef _SPIN_H_INCLUDED
#warning "Spin library is no longer required"
#endif
#define _SPIN_H_INCLUDED  // try to avoid spin library from loading.

#define ILI9341_USE_DMAMEM

// Allow us to enable or disable capabilities, particully Frame Buffer and
// Clipping for speed and size
#ifndef DISABLE_ILI9341_FRAMEBUFFER
#if defined(__MK66FX1M0__)  // T3.6
#define ENABLE_ILI9341_FRAMEBUFFER
#define SCREEN_DMA_NUM_SETTINGS \
    3                         // see if making it a constant value makes difference...
#elif defined(__MK64FX512__)  // T3.5
#define ENABLE_ILI9341_FRAMEBUFFER
#define SCREEN_DMA_NUM_SETTINGS \
    4  // see if making it a constant value makes difference...
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
#define ENABLE_ILI9341_FRAMEBUFFER
#define SCREEN_DMA_NUM_SETTINGS \
    3  // see if making it a constant value makes difference...
#endif
#endif

// Allow way to override using SPI

#ifdef __cplusplus
#include <DMAChannel.h>
#include <SPI.h>

#include <gfx_pixel.hpp>
#include <gfx_positioning.hpp>

#include "Arduino.h"
#endif
#include <stdint.h>

#include "ILI9341_fonts.h"
template <uint8_t SpiHost = 0>
struct teensy_tft_spi_driver {
    static SPIClass *_pspi;
    static SPIClass::SPI_Hardware_t *_spi_hardware;

    constexpr static const uint8_t _spi_num = SpiHost;  // Which buss is this spi on?

#if defined(KINETISK)
    static KINETISK_SPI_t *_pkinetisk_spi;
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
    static IMXRT_LPSPI_t *_pimxrt_spi;

#elif defined(KINETISL)
    static KINETISL_SPI_t *_pkinetisl_spi;
#endif
    static bool _initialized;
    static uint8_t _rst;
    static uint8_t _cs, _dc;
    static uint8_t pcs_data, pcs_command;
    static uint8_t _miso, _mosi, _sclk;
    inline static bool initialized() { return _initialized; }
    static bool initialize(uint8_t pin_cs, uint8_t pin_dc, uint8_t pin_rst = 0xFF) {
        if (_initialized) {
            return true;
        }
        _cs = pin_cs;
        _dc = pin_dc;
        _rst = pin_rst;
        switch (_spi_num) {
            case 0:
                _pspi = &SPI;
#ifdef KINETISK
                _pkinetisk_spi = &KINETISK_SPI0;  // Could hack our way to grab this from SPI
                                                  // object, but...
                _fifo_full_test = (3 << 12);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
                _pimxrt_spi = &IMXRT_LPSPI4_S;  // Could hack our way to grab this from SPI
                                                // object, but...
#else
                _pkinetisl_spi = &KINETISL_SPI0;
#endif
                break;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__IMXRT1062__) || defined(__MKL26Z64__)
            case 1:
                _pspi = &SPI1;
#ifdef KINETISK
                _pkinetisk_spi = &KINETISK_SPI1;  // Could hack our way to grab this from SPI
                                                  // object, but...
                _fifo_full_test = (0 << 12);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
                _pimxrt_spi = &IMXRT_LPSPI3_S;  // Could hack our way to grab this from SPI
                                                // object, but...
#else
                _pkinetisl_spi = &KINETISL_SPI1;
#endif
                break;
#if !defined(__MKL26Z64__)
            case 2:
                _pspi = &SPI2;
#ifdef KINETISK
                _pkinetisk_spi = &KINETISK_SPI2;  // Could hack our way to grab this from SPI
                                                  // object, but...
                _fifo_full_test = (0 << 12);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
                _pimxrt_spi = &IMXRT_LPSPI1_S;  // Could hack our way to grab this from SPI
                                                // object, but...
#endif
                break;
#endif
#endif
            default:
                return false;
        }
        // Hack to get hold of the SPI Hardware information...
        uint32_t *pa = (uint32_t *)((void *)_pspi);
        _spi_hardware = (SPIClass::SPI_Hardware_t *)(void *)pa[1];

        _pspi->begin();
#ifdef KINETISK
        if (_pspi->pinIsChipSelect(_cs, _dc)) {
            pcs_data = _pspi->setCS(_cs);
            pcs_command = pcs_data | _pspi->setCS(_dc);
        } else {
            // See if at least DC is on chipselect pin, if so try to limp along...
            if (_pspi->pinIsChipSelect(_dc)) {
                pcs_data = 0;
                pcs_command = pcs_data | _pspi->setCS(_dc);
                pinMode(_cs, OUTPUT);
                _csport = portOutputRegister(digitalPinToPort(_cs));
                _cspinmask = digitalPinToBitMask(_cs);
                *_csport |= _cspinmask;
            } else {
                pcs_data = 0;
                pcs_command = 0;
                Serial.println("teensy_gfx: Error not DC is not valid hardware CS pin");
                return gfx::gfx_result::invalid_argument;
            }
        }
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
        // Serial.println("   T4 setup CS/DC"); Serial.flush();
        pending_rx_count = 0;  // Make sure it is zero if we we do a second begin...
        _csport = portOutputRegister(_cs);
        _cspinmask = digitalPinToBitMask(_cs);
        pinMode(_cs, OUTPUT);
        DIRECT_WRITE_HIGH(_csport, _cspinmask);
        _spi_tcr_current = _pimxrt_spi->TCR;  // get the current TCR value

        // TODO:  Need to setup DC to actually work.
        if (_pspi->pinIsChipSelect(_dc)) {
            uint8_t dc_cs_index = _pspi->setCS(_dc);
            // Serial.printf("    T4 hardware DC: %x\n", dc_cs_index);
            _dcport = 0;
            _dcpinmask = 0;
            // will depend on which PCS but first get this to work...
            dc_cs_index--;  // convert to 0 based
            _tcr_dc_assert = LPSPI_TCR_PCS(dc_cs_index);
            _tcr_dc_not_assert = LPSPI_TCR_PCS(3);
        } else {
            // Serial.println("teensy_gfx: DC is not valid hardware CS pin");
            _dcport = portOutputRegister(_dc);
            _dcpinmask = digitalPinToBitMask(_dc);
            pinMode(_dc, OUTPUT);
            DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
            _tcr_dc_assert = LPSPI_TCR_PCS(0);
            _tcr_dc_not_assert = LPSPI_TCR_PCS(1);
        }
        maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));

#else
        // TLC
        pcs_data = 0;
        pcs_command = 0;
        pinMode(_cs, OUTPUT);
        _csport = portOutputRegister(digitalPinToPort(_cs));
        _cspinmask = digitalPinToBitMask(_cs);
        *_csport |= _cspinmask;
        pinMode(_dc, OUTPUT);
        _dcport = portOutputRegister(digitalPinToPort(_dc));
        _dcpinmask = digitalPinToBitMask(_dc);
        *_dcport |= _dcpinmask;
        _dcpinAsserted = 0;
#endif

        // toggle RST low to reset
        if (_rst < 255) {
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, HIGH);
            delay(5);
            digitalWrite(_rst, LOW);
            delay(20);
            digitalWrite(_rst, HIGH);
            delay(150);
        }
        _initialized = true;
        return true;
    }
///////////////////////////////
// BUGBUG:: reorganize this area better!
#if defined(KINETISK)
    // inline uint8_t sizeFIFO() {return _fifo_size; }
    static uint32_t _fifo_full_test;
    static void waitFifoNotFull(void) {
        uint32_t sr;
        uint32_t tmp __attribute__((unused));
        do {
            sr = _pkinetisk_spi->SR;
            if (sr & 0xF0)
                tmp = _pkinetisk_spi->POPR;  // drain RX FIFO
        } while ((uint32_t)(sr & (15 << 12)) > _fifo_full_test);
    }
    static void waitFifoEmpty(void) {
        uint32_t sr;
        uint32_t tmp __attribute__((unused));
        do {
            sr = _pkinetisk_spi->SR;
            if (sr & 0xF0)
                tmp = _pkinetisk_spi->POPR;  // drain RX FIFO
        } while ((sr & 0xF0F0) > 0);         // wait both RX & TX empty
    }
    static void waitTransmitComplete(void) {
        uint32_t tmp __attribute__((unused));
        while (!(_pkinetisk_spi->SR & SPI_SR_TCF))
            ;                        // wait until final output done
        tmp = _pkinetisk_spi->POPR;  // drain the final RX FIFO word
    }
    static void waitTransmitComplete(uint32_t mcr) {
        uint32_t tmp __attribute__((unused));
        while (1) {
            uint32_t sr = _pkinetisk_spi->SR;
            if (sr & SPI_SR_EOQF)
                break;  // wait for last transmit
            if (sr & 0xF0)
                tmp = _pkinetisk_spi->POPR;
        }
        _pkinetisk_spi->SR = SPI_SR_EOQF;
        _pkinetisk_spi->MCR = mcr;
        while (_pkinetisk_spi->SR & 0xF0) {
            tmp = _pkinetisk_spi->POPR;
        }
    }

#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
    static uint8_t pending_rx_count;  // hack ...
    static void waitFifoNotFull(void) {
        uint32_t tmp __attribute__((unused));
        do {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0) {
                tmp = _pimxrt_spi->RDR;  // Read any pending RX bytes in
                if (pending_rx_count)
                    pending_rx_count--;  // decrement count of bytes still levt
            }
        } while ((_pimxrt_spi->SR & LPSPI_SR_TDF) == 0);
    }
    static void waitFifoEmpty(void) {
        uint32_t tmp __attribute__((unused));
        do {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0) {
                tmp = _pimxrt_spi->RDR;  // Read any pending RX bytes in
                if (pending_rx_count)
                    pending_rx_count--;  // decrement count of bytes still levt
            }
        } while ((_pimxrt_spi->SR & LPSPI_SR_TCF) == 0);
    }
    static void waitTransmitComplete(void) {
        uint32_t tmp __attribute__((unused));
        //    digitalWriteFast(2, HIGH);

        while (pending_rx_count) {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0) {
                tmp = _pimxrt_spi->RDR;  // Read any pending RX bytes in
                pending_rx_count--;      // decrement count of bytes still levt
            }
        }
        _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF;  // Clear RX FIFO
    }
    static uint16_t waitTransmitCompleteReturnLast() {
        uint32_t val = 0;
        //    digitalWriteFast(2, HIGH);

        while (pending_rx_count) {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0) {
                val = _pimxrt_spi->RDR;  // Read any pending RX bytes in
                pending_rx_count--;      // decrement count of bytes still levt
            }
        }
        _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF;  // Clear RX FIFO
        return val;
    }
    static void waitTransmitComplete(uint32_t mcr) {
        // BUGBUG:: figure out if needed...
        waitTransmitComplete();
    }
#elif defined(KINETISL)
#endif
//////////////////////////////

// add support to allow only one hardware CS (used for dc)
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
    static uint32_t _cspinmask;
    static volatile uint32_t *_csport;
    static uint32_t _spi_tcr_current;
    static uint32_t _dcpinmask;
    static uint32_t _tcr_dc_assert;
    static uint32_t _tcr_dc_not_assert;
    static volatile uint32_t *_dcport;
#else
    static uint8_t _cspinmask;
    static volatile uint8_t *_csport;
#endif
#ifdef KINETISL
    static uint8_t _dcpinAsserted;
    static uint8_t _data_sent_not_completed;
    static volatile uint8_t *_dcport;
    static uint8_t _dcpinmask;
#endif

//. From Onewire utility files
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x

    static void DIRECT_WRITE_LOW(volatile uint32_t *base, uint32_t mask)
        __attribute__((always_inline)) {
        *(base + 34) = mask;
    }
    static void DIRECT_WRITE_HIGH(volatile uint32_t *base, uint32_t mask)
        __attribute__((always_inline)) {
        *(base + 33) = mask;
    }
#endif

    static void beginSPITransaction(uint32_t clock) __attribute__((always_inline)) {
        _pspi->beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
        if (!_dcport)
            _spi_tcr_current = _pimxrt_spi->TCR;  // Only if DC is on hardware CS
#endif
        if (_csport) {
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
            DIRECT_WRITE_LOW(_csport, _cspinmask);
#else
            *_csport &= ~_cspinmask;
#endif
        }
    }
    static void endSPITransaction() __attribute__((always_inline)) {
        if (_csport) {
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
            DIRECT_WRITE_HIGH(_csport, _cspinmask);
#else
            *_csport |= _cspinmask;
#endif
        }
        _pspi->endTransaction();
    }
#if defined(KINETISK)
    static void writecommand_cont(uint8_t c) __attribute__((always_inline)) {
        _pkinetisk_spi->PUSHR =
            c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
        waitFifoNotFull();
    }
    static void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
        _pkinetisk_spi->PUSHR =
            c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
        waitFifoNotFull();
    }
    static void writedata16_cont(uint16_t d) __attribute__((always_inline)) {
        _pkinetisk_spi->PUSHR =
            d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_CONT;
        waitFifoNotFull();
    }
    static void writecommand_last(uint8_t c) __attribute__((always_inline)) {
        uint32_t mcr = _pkinetisk_spi->MCR;
        _pkinetisk_spi->PUSHR =
            c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
        waitTransmitComplete(mcr);
    }
    static void writedata8_last(uint8_t c) __attribute__((always_inline)) {
        uint32_t mcr = _pkinetisk_spi->MCR;
        _pkinetisk_spi->PUSHR =
            c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
        waitTransmitComplete(mcr);
    }
    static void writedata16_last(uint16_t d) __attribute__((always_inline)) {
        uint32_t mcr = _pkinetisk_spi->MCR;
        _pkinetisk_spi->PUSHR =
            d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
        waitTransmitComplete(mcr);
    }
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
    constexpr static const uint32_t TCR_MASK =
        (LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK);
    static void maybeUpdateTCR(
        uint32_t requested_tcr_state) /*__attribute__((always_inline)) */ {
        if ((_spi_tcr_current & TCR_MASK) != requested_tcr_state) {
            bool dc_state_change = (_spi_tcr_current & LPSPI_TCR_PCS(3)) !=
                                   (requested_tcr_state & LPSPI_TCR_PCS(3));
            _spi_tcr_current = (_spi_tcr_current & ~TCR_MASK) | requested_tcr_state;
            // only output when Transfer queue is empty.
            if (!dc_state_change || !_dcpinmask) {
                while ((_pimxrt_spi->FSR & 0x1f))
                    ;
                _pimxrt_spi->TCR = _spi_tcr_current;  // update the TCR

            } else {
                waitTransmitComplete();
                if (requested_tcr_state & LPSPI_TCR_PCS(3))
                    DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
                else
                    DIRECT_WRITE_LOW(_dcport, _dcpinmask);
                _pimxrt_spi->TCR = _spi_tcr_current &
                                   ~(LPSPI_TCR_PCS(3) |
                                     LPSPI_TCR_CONT);  // go ahead and update TCR anyway?
            }
        }
    }

    // BUGBUG:: currently assumming we only have CS_0 as valid CS
    static void writecommand_cont(uint8_t c) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
        _pimxrt_spi->TDR = c;
        pending_rx_count++;  //
        waitFifoNotFull();
    }
    static void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = c;
        pending_rx_count++;  //
        waitFifoNotFull();
    }
    static void writedata16_cont(uint16_t d) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = d;
        pending_rx_count++;  //
        waitFifoNotFull();
    }
    static void writecommand_last(uint8_t c) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = c;
        //		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        pending_rx_count++;  //
        waitTransmitComplete();
    }
    static void writedata8_last(uint8_t c) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = c;
        //		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        pending_rx_count++;  //
        waitTransmitComplete();
    }
    static void writedata16_last(uint16_t d) __attribute__((always_inline)) {
        maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15));
        _pimxrt_spi->TDR = d;
        //		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        pending_rx_count++;  //
        waitTransmitComplete();
    }
    inline static void writeData(const uint8_t *data, size_t size) {
        while (--size) {
            writedata8_cont(*data++);
        }
        writedata8_last(*data);
    }
    inline static void writeData16(const uint16_t *data, size_t size) {
        while (--size) {
            writedata16_cont(*data++);
        }
        writedata16_last(*data);
    }

    inline static void writeDataRepeat(uint8_t data, size_t count) {
        while (--count) {
            writedata8_cont(data);
        }
        writedata8_last(data);
    }
    inline static void writeData16Repeat(uint16_t data, size_t count) {
        while (--count) {
            writedata16_cont(data);
        }
        writedata16_last(data);
    }

// Now lets see if we can read in multiple pixels
#ifdef KINETISK
    static void readData24to16(uint16_t *data, size_t size) {
        i

            uint8_t rgb[3];  // RGB bytes received from the display
        uint8_t rgbIdx = 0;
        uint32_t txCount = size + ((size + 1) / 2);  // num bytes to transmit
        uint32_t rxCount =
            txCount;  // number of bytes we will receive back from the display

        // transmit a DUMMY byte before the color bytes
        _pkinetisk_spi->PUSHR =
            0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;

        // skip values returned by the queued up transfers and the current in-flight
        // transfer
        uint32_t sr = _pkinetisk_spi->SR;
        uint8_t skipCount = ((sr >> 4) & 0xF) + ((sr >> 12) & 0xF) + 1;

        while (txCount || rxCount) {
            // transmit another byte if possible
            if (txCount && (_pkinetisk_spi->SR & 0xF000) <= _fifo_full_test) {
                txCount--;
                if (txCount) {
                    _pkinetisk_spi->PUSHR = READ_PIXEL_PUSH_BYTE | (pcs_data << 16) |
                                            SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
                } else {
                    _pkinetisk_spi->PUSHR = READ_PIXEL_PUSH_BYTE | (pcs_data << 16) |
                                            SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
                }
            }

            // receive another byte if possible, and either skip it or store the color
            if (rxCount && (_pkinetisk_spi->SR & 0xF0)) {
                rgb[rgbIdx] = _pkinetisk_spi->POPR;

                if (skipCount) {
                    skipCount--;
                } else {
                    rxCount--;
                    rgbIdx++;
                    if (rgbIdx == 3) {
                        rgbIdx = 0;
                        *data++ = ((rgb[0] & 0xF8) << 8) | ((rgb[1] & 0xFC) << 3) | (rgb[2] >> 3);
                    }
                }
            }
        }

        // wait for End of Queue
        while ((_pkinetisk_spi->SR & SPI_SR_EOQF) == 0)
            ;
        _pkinetisk_spi->SR = SPI_SR_EOQF;  // make sure it is clear
    }
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
    static void readData24to16(uint16_t *data, size_t size) {
        uint8_t rgb[3];  // RGB bytes received from the display
        uint8_t rgbIdx = 0;
        uint32_t txCount = size + ((size + 1) / 2);  // num bytes to transmit
        uint32_t rxCount =
            txCount;  // number of bytes we will receive back from the display

        while (txCount || rxCount) {
            // transmit another byte if possible
            if (txCount && (_pimxrt_spi->SR & LPSPI_SR_TDF)) {
                txCount--;
                if (txCount) {
                    _pimxrt_spi->TDR = 0;
                } else {
                    maybeUpdateTCR(_tcr_dc_not_assert |
                                   LPSPI_TCR_FRAMESZ(7));  // remove the CONTINUE...
                    while ((_pimxrt_spi->SR & LPSPI_SR_TDF) == 0)
                        ;  // wait if queue was full
                    _pimxrt_spi->TDR = 0;
                }
            }

            // receive another byte if possible, and either skip it or store the color
            if (rxCount && !(_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY)) {
                rgb[rgbIdx] = _pimxrt_spi->RDR;

                rxCount--;
                rgbIdx++;
                if (rgbIdx == 3) {
                    rgbIdx = 0;
                    *data++ = ((rgb[0] & 0xF8) << 8) | ((rgb[1] & 0xFC) << 3) | (rgb[2] >> 3);
                }
            }
        }
    }

#else

    // Teensy LC version
    static void readData24to16(uint16_t *data, size_t size) {
        uint8_t rgb[3];  // RGB bytes received from the display
        uint8_t rgbIdx = 0;
        uint32_t txCount = size + ((size + 1) / 2);  // num bytes to transmit
        uint32_t rxCount =
            txCount;  // number of bytes we will receive back from the display

        // Wait until that one returns, Could do a little better and double buffer but
        // this is easer for now.
        waitTransmitComplete();

// Since double buffer setup lets try keeping read/write in sync
#define RRECT_TIMEOUT 0xffff
#undef READ_PIXEL_PUSH_BYTE
#define READ_PIXEL_PUSH_BYTE 0  // try with zero to see...
        uint16_t timeout_countdown = RRECT_TIMEOUT;
        uint16_t dl_in;
        // Write out first byte:

        while (!(_pkinetisl_spi->S & SPI_S_SPTEF))
            ;  // Not worried that this can completely hang?
        _pkinetisl_spi->DL = READ_PIXEL_PUSH_BYTE;

        while (rxCount && timeout_countdown) {
            // Now wait until we can output something
            dl_in = 0xffff;
            if (rxCount > 1) {
                while (!(_pkinetisl_spi->S & SPI_S_SPTEF))
                    ;  // Not worried that this can completely hang?
                if (_pkinetisl_spi->S & SPI_S_SPRF)
                    dl_in = _pkinetisl_spi->DL;
                _pkinetisl_spi->DL = READ_PIXEL_PUSH_BYTE;
            }

            // Now wait until there is a byte available to receive
            while ((dl_in != 0xffff) && !(_pkinetisl_spi->S & SPI_S_SPRF) &&
                   --timeout_countdown)
                ;
            if (timeout_countdown) {  // Make sure we did not get here because of timeout
                rxCount--;
                rgb[rgbIdx] = (dl_in != 0xffff) ? dl_in : _pkinetisl_spi->DL;
                rgbIdx++;
                if (rgbIdx == 3) {
                    rgbIdx = 0;
                    *data++ = ((rgb[0] & 0xF8) << 8) | ((rgb[1] & 0xFC) << 3) | (rgb[2] >> 3);
                }
                timeout_countdown = timeout_countdown;
            }
        }

        // Debug code.
        /*	if (timeout_countdown == 0) {
                        Serial.print("RRect Timeout ");
                        Serial.println(rxCount, DEC);
                } */
    }
#endif

#elif defined(KINETISL)
    // Lets see how hard to make it work OK with T-LC
    static uint8_t _dcpinAsserted;
    static uint8_t _data_sent_not_completed;
    static void waitTransmitComplete() {
        while (_data_sent_not_completed) {
            uint16_t timeout_count = 0xff;  // hopefully enough
            while (!(_pkinetisl_spi->S & SPI_S_SPRF) && timeout_count--)
                ;  // wait
            uint8_t d __attribute__((unused));
            d = _pkinetisl_spi->DL;
            d = _pkinetisl_spi->DH;
            _data_sent_not_completed--;  // We hopefully received our data...
        }
    }
    static uint16_t waitTransmitCompleteReturnLast() {
        uint16_t d = 0;
        while (_data_sent_not_completed) {
            uint16_t timeout_count = 0xff;  // hopefully enough
            while (!(_pkinetisl_spi->S & SPI_S_SPRF) && timeout_count--)
                ;  // wait
            d = (_pkinetisl_spi->DH << 8) | _pkinetisl_spi->DL;
            _data_sent_not_completed--;  // We hopefully received our data...
        }
        return d;
    }

    static void setCommandMode() __attribute__((always_inline)) {
        if (!_dcpinAsserted) {
            waitTransmitComplete();
            *_dcport &= ~_dcpinmask;
            _dcpinAsserted = 1;
        }
    }

    static void setDataMode() __attribute__((always_inline)) {
        if (_dcpinAsserted) {
            waitTransmitComplete();
            *_dcport |= _dcpinmask;
            _dcpinAsserted = 0;
        }
    }

    static void outputToSPI(uint8_t c) {
        if (_pkinetisl_spi->C2 & SPI_C2_SPIMODE) {
            // Wait to change modes until any pending output has been done.
            waitTransmitComplete();
            _pkinetisl_spi->C2 = 0;  // make sure 8 bit mode.
        }
        while (!(_pkinetisl_spi->S & SPI_S_SPTEF))
            ;  // wait if output buffer busy.
        // Clear out buffer if there is something there...
        if ((_pkinetisl_spi->S & SPI_S_SPRF)) {
            uint8_t d __attribute__((unused));
            d = _pkinetisl_spi->DL;
            _data_sent_not_completed--;
        }
        _pkinetisl_spi->DL = c;      // output byte
        _data_sent_not_completed++;  // let system know we sent something
    }

    static void outputToSPI16(uint16_t data) {
        if (!(_pkinetisl_spi->C2 & SPI_C2_SPIMODE)) {
            // Wait to change modes until any pending output has been done.
            waitTransmitComplete();
            _pkinetisl_spi->C2 = SPI_C2_SPIMODE;  // make sure 8 bit mode.
        }
        uint8_t s;
        do {
            s = _pkinetisl_spi->S;
            // wait if output buffer busy.
            // Clear out buffer if there is something there...
            if ((s & SPI_S_SPRF)) {
                uint8_t d __attribute__((unused));
                d = _pkinetisl_spi->DL;
                d = _pkinetisl_spi->DH;
                _data_sent_not_completed--;  // let system know we sent something
            }

        } while (!(s & SPI_S_SPTEF) || (s & SPI_S_SPRF));

        _pkinetisl_spi->DL = data;       // output low byte
        _pkinetisl_spi->DH = data >> 8;  // output high byte
        _data_sent_not_completed++;      // let system know we sent something
    }

    static void writecommand_cont(uint8_t c) {
        setCommandMode();
        outputToSPI(c);
    }
    static void writedata8_cont(uint8_t c) {
        setDataMode();
        outputToSPI(c);
    }

    static void writedata16_cont(uint16_t c) {
        setDataMode();
        outputToSPI16(c);
    }

    static void writecommand_last(uint8_t c) {
        setCommandMode();
        outputToSPI(c);
        waitTransmitComplete();
    }
    static void writedata8_last(uint8_t c) {
        setDataMode();
        outputToSPI(c);
        waitTransmitComplete();
    }
    static void writedata16_last(uint16_t c) {
        setDataMode();
        outputToSPI16(c);
        waitTransmitComplete();
        _pkinetisl_spi->C2 = 0;  // Set back to 8 bit mode...
        _pkinetisl_spi->S;       // Read in the status;
    }

#endif
};
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_cspinmask = 0;
template <uint8_t SpiHost>
volatile uint32_t *teensy_tft_spi_driver<SpiHost>::_csport = nullptr;
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_spi_tcr_current = 0;
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_dcpinmask = 0;
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_tcr_dc_assert = 0;
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_tcr_dc_not_assert = 0;
template <uint8_t SpiHost>
volatile uint32_t *teensy_tft_spi_driver<SpiHost>::_dcport = nullptr;
#else
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_cspinmask = 0;
template <uint8_t SpiHost>
volatile uint8_t *teensy_tft_spi_driver<SpiHost>::_csport = nullptr;
#endif
#ifdef KINETISL
template <uint8_t SpiHost>
volatile uint8_t *teensy_tft_spi_driver<SpiHost>::_dcport = nullptr;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_dcpinmask = 0;
// Lets see how hard to make it work OK with T-LC
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_dcpinAsserted = 0;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_data_sent_not_completed = 0;
#endif
#if defined(KINETISK)
template <uint8_t SpiHost>
uint32_t teensy_tft_spi_driver<SpiHost>::_fifo_full_test;
template <uint8_t SpiHost>
KINETISK_SPI_t *teensy_tft_spi_driver<SpiHost>::_pkinetisk_spi = nullptr;
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
template <uint8_t SpiHost>
IMXRT_LPSPI_t *teensy_tft_spi_driver<SpiHost>::_pimxrt_spi = nullptr;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::pending_rx_count = 0;  // hack ...

#elif defined(KINETISL)
template <uint8_t SpiHost>
KINETISL_SPI_t *teensy_tft_spi_driver<SpiHost>::_pkinetisl_spi = nullptr;
#endif
template <uint8_t SpiHost>
bool teensy_tft_spi_driver<SpiHost>::_initialized = false;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_rst = 0xFF;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_cs = 0xFF;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_dc = 0xFF;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::pcs_data = 0;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::pcs_command = 0;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_miso = 0xFF;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_mosi = 0xFF;
template <uint8_t SpiHost>
uint8_t teensy_tft_spi_driver<SpiHost>::_sclk = 0xFF;
template <uint8_t SpiHost>
SPIClass* teensy_tft_spi_driver<SpiHost>::_pspi = nullptr;
template <uint8_t SpiHost>
SPIClass::SPI_Hardware_t* teensy_tft_spi_driver<SpiHost>::_spi_hardware = nullptr;
#define ILI9341_TFTWIDTH 240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP 0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID 0x04
#define ILI9341_RDDST 0x09

#define ILI9341_SLPIN 0x10
#define ILI9341_SLPOUT 0x11
#define ILI9341_PTLON 0x12
#define ILI9341_NORON 0x13

#define ILI9341_RDMODE 0x0A
#define ILI9341_RDMADCTL 0x0B
#define ILI9341_RDPIXFMT 0x0C
#define ILI9341_RDIMGFMT 0x0D
#define ILI9341_RDSELFDIAG 0x0F

#define ILI9341_INVOFF 0x20
#define ILI9341_INVON 0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON 0x29

#define ILI9341_CASET 0x2A
#define ILI9341_PASET 0x2B
#define ILI9341_RAMWR 0x2C
#define ILI9341_RAMRD 0x2E

#define ILI9341_PTLAR 0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL 0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT 0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR 0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1 0xC0
#define ILI9341_PWCTR2 0xC1
#define ILI9341_PWCTR3 0xC2
#define ILI9341_PWCTR4 0xC3
#define ILI9341_PWCTR5 0xC4
#define ILI9341_VMCTR1 0xC5
#define ILI9341_VMCTR2 0xC7

#define ILI9341_RDID1 0xDA
#define ILI9341_RDID2 0xDB
#define ILI9341_RDID3 0xDC
#define ILI9341_RDID4 0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/


#define CL(_r, _g, _b) ((((_r)&0xF8) << 8) | (((_g)&0xFC) << 3) | ((_b) >> 3))

#define sint16_t int16_t

#ifdef __cplusplus
// At all other speeds, _pspi->beginTransaction() will use the fastest available
// clock
#ifdef KINETISK
#define ILI9341_SPICLOCK 30000000
#define ILI9341_SPICLOCK_READ 2000000
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
#define ILI9341_SPICLOCK 30000000u
#define ILI9341_SPICLOCK_READ 2000000
#else
#define ILI9341_SPICLOCK 30000000
#define ILI9341_SPICLOCK_READ 2000000
#endif
namespace helpers_ili9341 {
static const uint8_t PROGMEM init_commands[] = {4, 0xEF, 0x03, 0x80, 0x02,
                                                4, 0xCF, 0x00, 0XC1, 0X30,
                                                5, 0xED, 0x64, 0x03, 0X12, 0X81,
                                                4, 0xE8, 0x85, 0x00, 0x78,
                                                6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
                                                2, 0xF7, 0x20,
                                                3, 0xEA, 0x00, 0x00,
                                                2, 0xC0, 0x23,        // Power control
                                                2, 0xC1, 0x10,        // Power control
                                                3, 0xC5, 0x3e, 0x28,  // VCM control
                                                2, 0xC7, 0x86,        // VCM control2
                                                2, 0x36, 0x48,        // Memory Access Control
                                                2, 0x3A, 0x55,        // pixel format
                                                3, 0xB1, 0x00, 0x18,
                                                4, 0xB6, 0x08, 0x82, 0x27,  // Display Function Control
                                                2, 0xF2, 0x00,              // Gamma Function Disable
                                                2, 0x26, 0x01,              // Gamma curve selected
                                                16, 0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E,
                                                0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,  // Set Gamma
                                                16, 0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31,
                                                0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,  // Set Gamma
                                                3, 0xb1, 0x00, 0x10,                             // FrameRate Control 119Hz
                                                0};
}
class teensy_gfx final {
    bool (*init)(uint8_t cs, uint8_t dc, uint8_t rst);
    bool (*isInit)();
    void (*beginSPITransaction)(uint32_t clock);
    void (*endSPITransaction)();
    void (*writecommand_cont)(uint8_t c);
    void (*writedata8_cont)(uint8_t c);
    void (*writedata16_cont)(uint16_t c);
    void (*writecommand_last)(uint8_t c);
    void (*writedata8_last)(uint8_t c);
    void (*writedata16_last)(uint16_t c);
    void (*writeData)(const uint8_t *data, size_t size);
    void (*writeData16)(const uint16_t *data, size_t size);
    void (*writeDataRepeat)(uint8_t data, size_t count);
    void (*writeData16Repeat)(uint16_t data, size_t count);
    void (*readData24to16)(uint16_t *data, size_t size);
    uint32_t _SPI_CLOCK;
    uint32_t _SPI_CLOCK_READ;
    uint8_t _cs;
    uint8_t _dc;
    uint8_t _rst;
    uint8_t m_rotation;
    void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) const
        __attribute__((always_inline)) {
        writecommand_cont(0x2a);  // Column addr set
        writedata16_cont(x0);     // XSTART
        writedata16_cont(x1);     // XEND
        writecommand_cont(0x2b);  // Row addr set
        writedata16_cont(y0);     // YSTART
        writedata16_cont(y1);     // YEND
    }

   public:
    using pixel_type = gfx::rgb_pixel<16>;
    using caps = gfx::gfx_caps<false, false, false, false, false, true, false>;
    inline gfx::size16 dimensions() const { return m_rotation&1?gfx::size16(320, 240):gfx::size16(240, 320); }
    inline gfx::rect16 bounds() const { return m_rotation&1?gfx::rect16(0, 0, 319, 239):gfx::rect16(0, 0, 239, 319); }
    gfx::gfx_result fill(const gfx::rect16 &bounds, pixel_type color) {
        gfx::gfx_result res = initialize();
        if (res != gfx::gfx_result::success) {
            return res;
        }
        gfx::rect16 b = bounds.normalize().crop(this->bounds());
        size_t w = (b.x2 - b.x1 + 1) * (b.y2 - b.y1 + 1);
        uint16_t c = color.native_value;
        beginSPITransaction(_SPI_CLOCK);
        setAddr(b.x1, b.y1, b.x2, b.y2);
        writecommand_cont(0x2C);
        writeData16Repeat(c,w);
        endSPITransaction();
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        gfx::gfx_result res = initialize();
        if (res != gfx::gfx_result::success) {
            return res;
        }
        if (!bounds().intersects(location)) {
            return gfx::gfx_result::success;
        }
        uint16_t c = color.native_value;
        beginSPITransaction(_SPI_CLOCK);
        setAddr(location.x, location.y, location.x, location.y);
        writecommand_cont(0x2c);
        writedata16_last(c);
        endSPITransaction();
        return gfx::gfx_result::success;
    }
    gfx::gfx_result point(gfx::point16 location, pixel_type *out_color) const {
        if (!initialized()) {
            return gfx::gfx_result::invalid_state;
        }
        if (out_color == nullptr) {
            return gfx::gfx_result::invalid_argument;
        }
        if (!bounds().intersects(location)) {
            out_color->native_value = pixel_type().native_value;
            return gfx::gfx_result::success;
        }
        uint16_t c;
        beginSPITransaction(_SPI_CLOCK_READ);

        setAddr(location.x,location.y,location.x,location.y);
        writecommand_cont(0x2E); // read from RAM
        writedata8_last(0); // dummy
        readData24to16(&c,1);
        endSPITransaction();
        out_color->native_value = c;
        return gfx::gfx_result::success;
    }
    inline gfx::gfx_result clear(const gfx::rect16 &bounds) {
        return fill(bounds, pixel_type());
    }
    teensy_gfx(uint8_t _SPI_HOST, uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, uint32_t write_speed = 30 * 1000 * 1000,uint32_t read_speed = 2*1000*1000) {
        m_rotation = 0;
        _SPI_CLOCK = write_speed;
        _SPI_CLOCK_READ = read_speed;
        switch (_SPI_HOST) {
            case 0: {
                using d = teensy_tft_spi_driver<0>;
                init = d::initialize;
                isInit = d::initialized;
                beginSPITransaction = d::beginSPITransaction;
                endSPITransaction = d::endSPITransaction;
                writecommand_cont = d::writecommand_cont;
                writedata8_cont = d::writedata8_cont;
                writedata16_cont = d::writedata16_cont;
                writecommand_last = d::writecommand_last;
                writeData = d::writeData;
                writeData16 = d::writeData16;
                writeDataRepeat = d::writeDataRepeat;
                writeData16Repeat = d::writeData16Repeat;
                readData24to16 = d::readData24to16;
            } break;
            case 1: {
                using d = teensy_tft_spi_driver<1>;
                init = d::initialize;
                isInit = d::initialized;
                beginSPITransaction = d::beginSPITransaction;
                endSPITransaction = d::endSPITransaction;
                writecommand_cont = d::writecommand_cont;
                writedata8_cont = d::writedata8_cont;
                writedata16_cont = d::writedata16_cont;
                writecommand_last = d::writecommand_last;
                writeData = d::writeData;
                writeData16 = d::writeData16;
                writeDataRepeat = d::writeDataRepeat;
                writeData16Repeat = d::writeData16Repeat;
                readData24to16 = d::readData24to16;
            } break;
            case 2: {
                using d = teensy_tft_spi_driver<2>;
                init = d::initialize;
                isInit = d::initialized;
                beginSPITransaction = d::beginSPITransaction;
                endSPITransaction = d::endSPITransaction;
                writecommand_cont = d::writecommand_cont;
                writedata8_cont = d::writedata8_cont;
                writedata16_cont = d::writedata16_cont;
                writecommand_last = d::writecommand_last;
                writeData = d::writeData;
                writeData16 = d::writeData16;
                writeDataRepeat = d::writeDataRepeat;
                writeData16Repeat = d::writeData16Repeat;
                readData24to16 = d::readData24to16;
            } break;
        }
        _cs = _CS;
        _dc = _DC;
        _rst = _RST;
    }
    inline bool initialized() const {
        return isInit();
    }
    gfx::gfx_result initialize() {
        if (isInit()) {
            return gfx::gfx_result::success;
        }
        if (!init(_cs, _dc, _rst)) {
            return gfx::gfx_result::invalid_argument;
        }
        beginSPITransaction(_SPI_CLOCK / 4);
        const uint8_t *addr = helpers_ili9341::init_commands;
        while (1) {
            uint8_t count = *addr++;
            if (count-- == 0)
                break;
            writecommand_cont(*addr++);
            while (count-- > 0) {
                writedata8_cont(*addr++);
            }
        }
        writecommand_last(0x11);  // Exit Sleep
        endSPITransaction();
        delay(120);
        beginSPITransaction(_SPI_CLOCK);
        writecommand_last(0x29);  // Display on
        endSPITransaction();
        return gfx::gfx_result::success;
    }
};

#ifndef ILI9341_swap
#define ILI9341_swap(a, b) \
    {                      \
        typeof(a) t = a;   \
        a = b;             \
        b = t;             \
    }
#endif

#endif  // __cplusplus

#endif
