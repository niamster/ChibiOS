/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    templates/mac_lld.c
 * @brief   MAC Driver subsystem low level driver source template.
 *
 * @addtogroup MAC
 * @{
 */

#include <string.h>

#include "ch.h"
#include "hal.h"
#include "mii.h"

#if HAL_USE_MAC || defined(__DOXYGEN__)

#define DBG_LEVEL 0
void dbgprintf(const char *fmt, ...);
#define Msg(_f_, ...) dbgprintf(_f_, ## __VA_ARGS__)
#define Tr1(_f_, ...) if(DBG_LEVEL>0) dbgprintf("%s: " _f_, __func__,  ## __VA_ARGS__)
#define Tr2(_f_, ...) if(DBG_LEVEL>1) dbgprintf("%s: " _f_, __func__,  ## __VA_ARGS__)
#define Tr3(_f_, ...) if(DBG_LEVEL>2) dbgprintf("%s: " _f_, __func__,  ## __VA_ARGS__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define BIT_(_n_)           (1<<(_n_))
#define MASK_(_n_,_s_)      (((1<<(_s_))-1)<<(_n_))
#define GET_(_n_,_s_,_v_)   (((_v_)&MASK_(_n_,_s_))>>(_n_))
#define SET_(_n_,_v_)       ((_v_)<<(_n_))

#define ETHCON1_PTV_SET(_v_)        SET_(16,(_v_)&0xFFFF)   // PAUSE Timer Value
#define ETHCON1_ON                  BIT_(15)                // Ethernet ON
#define ETHCON1_SIDL                BIT_(13_)               // Ethernet Stop in Idle Mode
#define ETHCON1_TXRTS               BIT_(9)                 // Transmit Request to Send
#define ETHCON1_RXEN                BIT_(8)                 // Receive Enable
#define ETHCON1_AUTOFC              BIT_(7)                 // Automatic Flow Control
#define ETHCON1_MANFC               BIT_(4)                 // Manual Flow Control
#define ETHCON1_BUFCDEC             BIT_(0)                 // Descriptor Buffer Count Decrement

#define ETHRXFC_HTEN                BIT_(15)                // Enable Hash Table Filtering bit
#define ETHRXFC_MPEN                BIT_(14)                // Magic Packet Enable
#define ETHRXFC_NOTPM               BIT_(12)                // Pattern Match Inversion
#define ETHRXFC_PMMODE              MASK_(8,4)              // Pattern Match Mode:
#define ETHRXFC_PMMODE_0            SET_(8,0)
#define ETHRXFC_PMMODE_1            SET_(8,1)
#define ETHRXFC_PMMODE_2            SET_(8,2)
#define ETHRXFC_PMMODE_3            SET_(8,3)
#define ETHRXFC_PMMODE_4            SET_(8,4)
#define ETHRXFC_PMMODE_5            SET_(8,5)
#define ETHRXFC_PMMODE_6            SET_(8,6)
#define ETHRXFC_PMMODE_7            SET_(8,7)
#define ETHRXFC_PMMODE_8            SET_(8,8)
#define ETHRXFC_PMMODE_9            SET_(8,9)
#define ETHRXFC_CRCERREN            BIT_(7)                 // CRC Error Collection Enable
#define ETHRXFC_CRCOKEN             BIT_(6)                 // CRC OK Enable
#define ETHRXFC_RUNTERREN           BIT_(5)                 // Runt Error Collection Enable
#define ETHRXFC_RUNTEN              BIT_(4)                 // Runt Enable
#define ETHRXFC_UCEN                BIT_(3)                 // Unicast Enable
#define ETHRXFC_NOTMEEN             BIT_(2)                 // Not Me Unicast Enable
#define ETHRXFC_MCEN                BIT_(1)                 // Multicast Enable
#define ETHRXFC_BCEN                BIT_(0)                 // Broadcast Enable

#define ETHRXWM_RXFWM_SET(_v_)      SET_(16,(_v_)&0xFF)     // Receive Full Watermark
#define ETHRXWM_RXEWM_SET(_v_)      SET_(0,(_v_)&0xFF)      // Receive Empty Watermark

// the following bits are for both IEN and IRQ registers
#define ETHINT_TXBUSE               BIT_(14)                // Transmit BVCI Bus Error
#define ETHINT_RXBUSE               BIT_(13)                // Receive BVCI Bus Error
#define ETHINT_EWMARK               BIT_(9)                 // Empty Watermark
#define ETHINT_FWMARK               BIT_(8)                 // Full Watermark
#define ETHINT_RXDONE               BIT_(7)                 // Receiver Done
#define ETHINT_PKTPEND              BIT_(6)                 // Packet Pending
#define ETHINT_RXACT                BIT_(5)                 // RX Activity
#define ETHINT_TXDONE               BIT_(3)                 // Transmitter Done
#define ETHINT_TXABORT              BIT_(2)                 // Transmitter Abort
#define ETHINT_RXBUFNA              BIT_(1)                 // Receive Buffer Not Available
#define ETHINT_RXOVFLW              BIT_(0)                 // Receive FIFO Overflow

#define ETHSTAT_BUFCNT_GET(_r_)     GET_(15,8,_r_)          // Packet Buffer Count
#define ETHSTAT_ETHBUSY             BIT_(7)                 // Ethernet Module busy
#define ETHSTAT_TXBUSY              BIT_(6)                 // Transmit Busy
#define ETHSTAT_RXBUSY              BIT_(5)                 // Receive Busy

#define MACCFG1_SOFTRESET           BIT_(15)                // Soft Reset
#define MACCFG1_SIMRESET            BIT_(14)                // Simulation Reset
#define MACCFG1_RESETRMCS           BIT_(11)                // Reset MCS/RX
#define MACCFG1_RESETRFUN           BIT_(10)                // Reset RX Function
#define MACCFG1_RESETTMCS           BIT_(9)                 // Reset MCS/TX
#define MACCFG1_RESETTFUN           BIT_(8)                 // Reset TX Function
#define MACCFG1_LOOPBACK            BIT_(4)                 //  MAC Loopback mode
#define MACCFG1_TXPAUSE             BIT_(3)                 // MAC TX Flow Control
#define MACCFG1_RXPAUSE             BIT_(2)                 // MAC RX Flow Control
#define MACCFG1_PASSALL             BIT_(1)                 // MAC Pass all Receive Frames
#define MACCFG1_RXENABLE            BIT_(0)                 // MAC Receive Enable

#define MACCFG2_EXCESSDFR           BIT_(14)                //  Excess Defer
#define MACCFG2_BPNOBKOFF           BIT_(13)                // Backpressure/No Backoff
#define MACCFG2_NOBKOFF             BIT_(12)                // No Backoff
#define MACCFG2_LONGPRE             BIT_(9)                 // Long Preamble Enforcement
#define MACCFG2_PUREPRE             BIT_(8)                 // Pure Preamble Enforcement
#define MACCFG2_AUTOPAD             BIT_(7)                 // Auto Detect Pad Enable
#define MACCFG2_VLANPAD             BIT_(6)                 // VLAN Pad Enable
#define MACCFG2_PADENABLE           BIT_(5)                 // Pad/CRC Enable
#define MACCFG2_CRCENABLE           BIT_(4)                 // CRC Enable
#define MACCFG2_DELAYCRC            BIT_(3)                 // Delayed CRC
#define MACCFG2_HUGEFRM             BIT_(2)                 // Huge Frame enable
#define MACCFG2_LENGTHCK            BIT_(1)                 // Frame Length checking
#define MACCFG2_FULLDPLX            BIT_(0)                 // Full-Duplex Operation

#define MACSUPP_RESETRMII           BIT_(11)                // Reset RMII Logic
#define MACSUPP_SPEEDRMII           BIT_(8)                 // RMII Speed

#define MACTEST_TESTBP              BIT_(2)                 // Test Backpressure
#define MACTEST_TESTPAUSE           BIT_(1)                 // Test PAUSE
#define MACTEST_SHRTQNTA            BIT_(0)                 // Shortcut PAUSE Quanta

#define MACMCFG_RESETMGMT           BIT_(15)                // Test Reset MII Management
#define MACMCFG_CLKSEL              MASK_(2,4)              // MII Management Clock Select:
#define MACMCFG_CLKSEL_DIV4         SET_(2,0)               //  SYSCLK divided by 4
#define MACMCFG_CLKSEL_DIV6         SET_(2,2)               //  SYSCLK divided by 6
#define MACMCFG_CLKSEL_DIV8         SET_(2,3)               //  SYSCLK divided by 8
#define MACMCFG_CLKSEL_DIV10        SET_(2,4)               //  SYSCLK divided by 10
#define MACMCFG_CLKSEL_DIV14        SET_(2,5)               //  SYSCLK divided by 14
#define MACMCFG_CLKSEL_DIV20        SET_(2,6)               //  SYSCLK divided by 20
#define MACMCFG_CLKSEL_DIV28        SET_(2,7)               //  SYSCLK divided by 28
#define MACMCFG_CLKSEL_DIV40        SET_(2,8)               //  SYSCLK divided by 40
#define MACMCFG_NOPRE               BIT_(1)                 // Suppress Preamble
#define MACMCFG_SCANINC             BIT_(0)                 // Scan Increment

#define MACMCMD_SCAN                BIT_(1)                 // MII Management Scan Mode
#define MACMCMD_READ                BIT_(0)                 // MII Management Read Command

#define MACMADR_SET(_p_,_r_)        (SET_(8,(_p_)&0x1F) | SET_(0,(_r_)&0x1F)) // MII Management PHY and reg Addresses

#define MACMIND_LINKFAIL            BIT_(3)                 // Link Fail
#define MACMIND_NOTVALID            BIT_(2)                 // MII Management Read Data Not Valid
#define MACMIND_SCAN                BIT_(1)                 // MII Management Scanning
#define MACMIND_MIIMBUSY            BIT_(0)                 // MII Management Busy

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief MAC1 driver identifier.
 */
#if PLATFORM_MAC_USE_MAC1 || defined(__DOXYGEN__)

MACDriver ETHD1 = {
  .irq  = EIC_ETH_IRQ,
  .base = _ETH_BASE_ADDRESS,
};

#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of ETH registers layout.
 */
typedef volatile struct {
  // offset 0x000
  PicReg con1;
  PicReg con2;
  PicReg txst;
  PicReg rxst;
  PicReg ht[2];
  PicReg pmm[2];
  PicReg pmcs;
  PicReg pmo;
  PicReg rxfc;
  PicReg rxwm;
  PicReg ien;
  PicReg irq;
  PicReg stat;

  PicReg noreg_0F0;

  // offset 0x100
  PicReg rxovflow;
  PicReg frmtxok;
  PicReg scolfrm;
  PicReg mcolfrm;
  PicReg frmrxok;
  PicReg fcserr;
  PicReg algnerr;

  PicReg noreg_170[9];

  // offset 0x200
  PicReg cfg1;
  PicReg cfg2;
  PicReg ipgt;
  PicReg ipgr;
  PicReg clrt;
  PicReg maxf;
  PicReg supp;
  PicReg test;
  PicReg mcfg;
  PicReg mcmd;
  PicReg madr;
  PicReg mwtd;
  PicReg mrdd;
  PicReg mind;

  PicReg noreg_2E0[2];

  // offsett 0x300
  PicReg sa[3];
} ethRegs;

/**
 * @brief   Type of ETH DMA descriptor
 */
typedef struct _pic32eth_desc {
  union {
    struct { // both TX and RX descriptors
      uint32_t res1 : 7;    // reseved
      uint32_t eown : 1;    // Ethernet Controller Own

      uint32_t npv  : 1;    // NEXT_ED pointer valid
      uint32_t usr0 : 1;    // user defined bits
      uint32_t usr1 : 1;
      uint32_t usr2 : 1;
      uint32_t usr3 : 1;
      uint32_t usr4 : 1;
      uint32_t usr5 : 1;
      uint32_t usr6 : 1;

      uint32_t count: 11;   // byte count (data bytes to transmit, 1..2047)
      uint32_t usr7 : 1;    // user defined bits
      uint32_t usr8 : 1;
      uint32_t usr9 : 1;
      uint32_t eop  : 1;    // end of packet
      uint32_t sop  : 1;    // start of packet
    };
    uint32_t Off0;
  };

  union {
    struct { // both TX and RX descriptors
      uint32_t addr;        // data buffer address
    };
    uint32_t Off4;
  };

  union {
    struct { // TX Descriptor
      uint32_t tx_total     : 16;   // total bytes transmitted, including collided attempts

      uint32_t tx_control   : 1;    // control frame transmitted
      uint32_t tx_pause     : 1;    // 'pause' control frame transmitted
      uint32_t tx_bprs      : 1;    // transmit backpressure applied
      uint32_t tx_vlan      : 1;    // VLAN tagged frame
      uint32_t tx_res       : 4;

      uint32_t tx_usr       : 8;    // user defined bits
    };
    struct { // RX Descriptor
      uint32_t rx_checksum  : 16;   // packet checksum

      uint32_t rx_usr       : 8;    // user defined bits

      uint32_t rx_runt      : 1;    // runt packet (<64 bytes)
      uint32_t rx_notmatch  : 1;    // NOT Unicast match & NOT Multicast match
      uint32_t rx_hashtable : 1;    // Hash table match
      uint32_t rx_magic     : 1;    // Magic packet match
      uint32_t rx_pattern   : 1;    // Pattern match
      uint32_t rx_unicast   : 1;    // Unicast match
      uint32_t rx_broadcast : 1;    // Broadcast match
      uint32_t rx_multicast : 1;    // Multicast match
    };
    uint32_t Off8;
  };

  union {
    struct { // TX Descriptor
      uint32_t tx_count     : 16;   // total bytes in frame, not including collided attempts

      uint32_t tx_coll      : 4;    // collision count
      uint32_t tx_err_crc   : 1;    // CRC error
      uint32_t tx_err_lce   : 1;    // Length check error
      uint32_t tx_err_loor  : 1;    // Length Out Of Range
      uint32_t tx_done      : 1;    // transmit done

      uint32_t tx_multicast : 1;    // destination address was multicast address
      uint32_t tx_broadcast : 1;    // destination address was broadcast address
      uint32_t tx_defer     : 1;    // packet was deferred
      uint32_t tx_exc_defer : 1;    // packet was excessively deferred
      uint32_t tx_collabt   : 1;    // transmit aborted due to collision count > HAL_PIC32MX_MACCLRT
      uint32_t tx_latecoll  : 1;    // transmit late collision
      uint32_t tx_giant     : 1;    // byte count was greater than HAL_PIC32MX_MACMAXF
      uint32_t tx_underrun  : 1;    // failed to transmit whole packet to MAC
    };
    struct { // RX Descriptor
      uint32_t rx_count     : 16;   // received byte count (frame length)

      uint32_t rx_rsv16     : 1;    // receive status vector
      uint32_t rx_rsv17     : 1;    // receive status vector
      uint32_t rx_rsv18     : 1;    // receive status vector
      uint32_t rx_rsv19     : 1;    // receive status vector
      uint32_t rx_err_crc   : 1;    // CRC error
      uint32_t rx_err_lce   : 1;    // Length Check error
      uint32_t rx_err_loor  : 1;    // Length Out Of Range
      uint32_t rx_done      : 1;    // received OK

      uint32_t rx_mc        : 1;    // received multicast packet
      uint32_t rx_bc        : 1;    // receive broadcast packet
      uint32_t rx_dn        : 1;    // dribble nibble received
      uint32_t rx_control   : 1;    // received control frame
      uint32_t rx_pause     : 1;    // received 'pause' control frame
      uint32_t rx_unknown   : 1;    // received unknown control frame
      uint32_t rx_vlan      : 1;    // received VLAN-tagged frame
      uint32_t rx_res       : 1;    // reserved
    };
    uint32_t OffC;
  };
} pic32eth_desc;

typedef uint32_t pic32eth_desc_next;  // next descriptor address

// transmit DMA descriptor list
//  all descriptors have npv=0
//  the last descriptor terminates the list, eown=0
#define PIC32_TX_DMA_DESCRIPTORS (MAC_TRANSMIT_DESCRIPTORS * PIC32_MAC_BUF_PER_FRAME)
static struct _pic32eth_tx {
  pic32eth_desc desc[PIC32_TX_DMA_DESCRIPTORS];
  pic32eth_desc_next loop;
} pic32eth_tx;

// receive DMA descriptor list
//  all descritors but last have npv=0
//  the last descriptor has npv=1 and followed by pic32eth_desc_next pointing to the first descriptor
#define PIC32_RX_DMA_DESCRIPTORS (MAC_RECEIVE_DESCRIPTORS * PIC32_MAC_BUF_PER_FRAME)
static struct _pic32eth_rx {
  pic32eth_desc desc[PIC32_RX_DMA_DESCRIPTORS];
  pic32eth_desc_next loop;
} pic32eth_rx;

// buffers //TODO: dyn buffer assignment to rx/tx DMA descriptors
typedef uint8_t pic32eth_buffer[MAC_BUFFER_SIZE];
static pic32eth_buffer pic32eth_tx_buf[PIC32_TX_DMA_DESCRIPTORS];
static pic32eth_buffer pic32eth_rx_buf[PIC32_RX_DMA_DESCRIPTORS];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// pic32eth_desc_init - initialize descriptor lists
static void pic32eth_desc_init(MACDriver *macp) {
  int i;

  Msg("eth: Tx %d(%d)  Rx %d(%d)  Buf %d  Mem: %d bytes\n",
    MAC_TRANSMIT_DESCRIPTORS, PIC32_TX_DMA_DESCRIPTORS,
    MAC_RECEIVE_DESCRIPTORS, PIC32_RX_DMA_DESCRIPTORS,
    MAC_BUFFER_SIZE,
    sizeof(pic32eth_tx) + sizeof(pic32eth_rx) +
    sizeof(pic32eth_rx_buf) + sizeof(pic32eth_tx_buf) );

  // Init RX DMA descriptor ring with all but last one owned by the controller
  memset(&pic32eth_rx, 0, sizeof(pic32eth_rx));

  for( i=0; i<PIC32_RX_DMA_DESCRIPTORS; i++ ) {
    pic32eth_desc* d = &pic32eth_rx.desc[i];

    memset( &pic32eth_rx_buf[i], 0, MAC_BUFFER_SIZE );

    d->addr = MIPS_PHYSICAL((uint32_t)&pic32eth_rx_buf[i]);

    // all descriptors but last are initially owned by the Controller
    d->eown = 1;
  }

  pic32eth_rx.desc[PIC32_RX_DMA_DESCRIPTORS-1].npv = 1;
  pic32eth_rx.desc[PIC32_RX_DMA_DESCRIPTORS-1].eown = 0;
  pic32eth_rx.loop = MIPS_PHYSICAL((uint32_t)&pic32eth_rx.desc[0]);

  macp->rxStart = 0;
  macp->rxEnd = PIC32_RX_DMA_DESCRIPTORS-1;

  // Init TX DMA descriptor ring, all descriptors are owned by the driver
  memset(&pic32eth_tx, 0, sizeof(pic32eth_tx));

  for( i=0; i<PIC32_TX_DMA_DESCRIPTORS; i++ ) {
    pic32eth_desc* d = &pic32eth_tx.desc[i];

    memset( &pic32eth_tx_buf[i], 0, MAC_BUFFER_SIZE );

    d->addr = MIPS_PHYSICAL((uint32_t)&pic32eth_tx_buf[i]);
  }

  pic32eth_tx.desc[PIC32_TX_DMA_DESCRIPTORS-1].npv = 1;
  pic32eth_tx.loop = MIPS_PHYSICAL((uint32_t)&pic32eth_tx.desc[0]);

  macp->txNext = 0;
  macp->txAvail = PIC32_TX_DMA_DESCRIPTORS;
}

// pic32eth_ctl_init - initialize Controller
static void pic32eth_ctl_init(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3("\n");

  //  disable interrupts
  eicDisableIrq(macp->irq);

  //  disable controller
  regs->con1.clear = 0
    | ETHCON1_ON
    | ETHCON1_RXEN
    | ETHCON1_TXRTS
    ;

  //  poll activity
  while( regs->stat.reg & ETHSTAT_ETHBUSY )
    halPolledDelay(MS2RTT(10));

  //  clear interrupt flag
  eicAckIrq(macp->irq);

  // disable all interrupts
  regs->ien.reg = 0;

  // clear start addresses
  regs->txst.reg = 0;
  regs->rxst.reg = 0;

  // enable controller pins to allow PHY access
  regs->con1.set = ETHCON1_ON;
  Tr3( "ETHCON1: 0x%X\n", regs->con1.reg );
}

// start Controller
static void pic32eth_ctl_start(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "\n");

  // flow control
  regs->rxwm.reg = ETHRXWM_RXFWM_SET(PIC32_RX_DMA_DESCRIPTORS/2);
  Tr3( "ETHRXWM: 0x%X\n", regs->rxwm.reg );

  regs->con1.set = ETHCON1_AUTOFC;
  Tr3( "ETHCON1: 0x%X\n", regs->con1.reg );

  // RX buffer size
  regs->con2.reg = MAC_BUFFER_SIZE;
  Tr3( "ETHCON2: 0x%X\n", regs->con2.reg );

  // packet Filtering
  regs->rxfc.reg = 0
    | ETHRXFC_BCEN          // Broadcast Enable
    | ETHRXFC_UCEN          // Unicast Enable
    | ETHRXFC_CRCOKEN       // CRC OK Enable
    ;
  Tr3( "ETHRXFC: 0x%X\n", regs->rxfc.reg );

  regs->rxst.reg = MIPS_PHYSICAL((uint32_t)&pic32eth_rx.desc[0]);
  Tr3( "ETHRXST: 0x%X\n", regs->rxst.reg );

  // enable receiver interrupts
  regs->ien.set = 0
    | ETHINT_RXBUSE     // Receive BVCI Bus Error
    | ETHINT_RXDONE     // Receiver Done
    | ETHINT_RXBUFNA    // Receive Buffer Not Available
    | ETHINT_RXOVFLW    // Receive FIFO Overflow
    ;
  Tr3( "ETHIEN: 0x%X\n", regs->ien.reg );

  // enable receiver
  regs->con1.set = ETHCON1_RXEN;
  Tr3( "ETHCON1: 0x%X\n", regs->con1.reg );

  //  enable interrupts
  eicEnableIrq(macp->irq);
}

// start transmitter
static void pic32eth_tx_start(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "\n");

  // enable transmitter interrupts
  regs->ien.set = 0
    | ETHINT_TXBUSE     // Transmit BVCI Bus Error
    | ETHINT_TXDONE     // Transmitter Done
    | ETHINT_TXABORT    // Transmitter Abort
    ;
  Tr3( "ETHIEN: 0x%08X\n", regs->ien.reg );

  // enable transmitter
  regs->con1.set = ETHCON1_TXRTS;
  Tr3( "ETHCON1: 0x%08X\n", regs->con1.reg );
}

// stop transmitter
static void pic32eth_tx_stop(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "\n");

  // disable transmitter
  regs->con1.clear = ETHCON1_TXRTS;
  Tr3( "ETHCON1: 0x%X\n", regs->con1.reg );

  // disable transmitter interrupts
  regs->ien.clear = 0
    | ETHINT_TXBUSE     // Transmit BVCI Bus Error
    | ETHINT_TXDONE     // Transmitter Done
    | ETHINT_TXABORT    // Transmitter Abort
    ;
  Tr3( "ETHIEN: 0x%X\n", regs->ien.reg );
}

// initialize MII Management interface
static void pic32eth_miim_init(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "\n");

  // reset MIIM (management)
  regs->mcfg.set = MACMCFG_RESETMGMT;
  halPolledDelay(MS2RTT(1));
  regs->mcfg.clear = MACMCFG_RESETMGMT;

  // set MDC (Management Data Clock) to slowest possible
  regs->mcfg.clear = MACMCFG_CLKSEL;
  regs->mcfg.set = MACMCFG_CLKSEL_DIV40;
}

// Read PHY register
static uint16_t pic32eth_miim_read(MACDriver *macp, uint8_t phy, uint8_t reg) {
  ethRegs *regs = (ethRegs *)macp->base;
  uint16_t ret = 0;

  Tr3( "\n");

  regs->madr.reg = MACMADR_SET(phy,reg);
  Tr3( "MACMADR: 0x%X\n", regs->madr.reg );

  regs->mcmd.set = MACMCMD_READ;
  while( regs->mind.reg & MACMIND_MIIMBUSY ); // TODO: timeout
  regs->mcmd.clear = MACMCMD_READ;

  ret = regs->mrdd.reg & 0xFFFF;
  Tr3( "MACMRDD: 0x%X\n", ret );

  return ret;
}

// Write PHY register
static void pic32eth_miim_write(MACDriver *macp, uint8_t phy, uint8_t reg, uint16_t data) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "\n");

  regs->madr.reg = MACMADR_SET(phy,reg);
  regs->mwtd.reg = data;
  while( regs->mind.reg & MACMIND_MIIMBUSY ); // TODO: timeout
}

// pic32eth_mac_init - initialize MAC
static void pic32eth_mac_init(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;

  Tr3( "regs 0x%X\n", regs);

  //  reset MAC
  Tr3( "reset MAC\n");
  regs->cfg1.set = MACCFG1_SOFTRESET;
  halPolledDelay(MS2RTT(1));
  regs->cfg1.clear = MACCFG1_SOFTRESET;

  // init IO pins
  Tr3( "init IO pins\n");
  if( macp->mii ) {  // MII interface
    // TODO: implement
    chDbgPanic("PIC32ETH: MII interface not supported");
  } else {           // RMII interface
    if( macp->io ) {    // Default IO pins
      // TODO: implement
      chDbgPanic("PIC32ETH: default IO pins not supported");
    } else {            // Alternate IO pins
      palSetGroupMode(IOPORTD, BIT_(8), 0, PAL_MODE_INPUT);
      palSetGroupMode(IOPORTE, BIT_(8) | BIT_(9), 0, PAL_MODE_INPUT);
      palSetGroupMode(IOPORTG, BIT_(8) | BIT_(9) | BIT_(15), 0, PAL_MODE_INPUT);
      palSetGroupMode(IOPORTA, BIT_(15), 0, PAL_MODE_OUTPUT);
      palSetGroupMode(IOPORTD, BIT_(11) | BIT_(14) | BIT_(15), 0, PAL_MODE_OUTPUT);
    }
  }

  // reset RMII and set its speed
  if (!macp->mii) {
    Tr3( "reset RMII\n");

    regs->supp.set = MACSUPP_RESETRMII;
    halPolledDelay(MS2RTT(1));
    regs->supp.clear = MACSUPP_RESETRMII;

    regs->supp.set = MACSUPP_SPEEDRMII;
  }

  // Init Management interface
  pic32eth_miim_init(macp);
}

// pic32eth_mac_start - start MAC
static void pic32eth_mac_start(MACDriver *macp) {
  ethRegs *regs = (ethRegs *)macp->base;
  uint8_t* addr = macp->config->mac_address;

  Tr3( "\n");

  // read PHY ID
  macp->phyID = pic32eth_miim_read(macp, macp->phy, 2);
  macp->phyID <<= 16;
  macp->phyID += pic32eth_miim_read(macp, macp->phy, 3);

  Tr1( "ETH1: PHY ID 0x%X\n", macp->phyID );

  regs->cfg1.set = 0
    | MACCFG1_RXENABLE      // MAC Receive Enable
    | MACCFG1_RXPAUSE       // MAC RX Flow Control
    | MACCFG1_TXPAUSE       // MAC TX Flow Control
    ;
  Tr2( "MACCFG1: 0x%X\n", regs->cfg1.reg );

  regs->cfg2.reg = 0
    | MACCFG2_FULLDPLX      // Full-Duplex Operation
    | MACCFG2_CRCENABLE     // CRC Enable
    | MACCFG2_PADENABLE     // Pad/CRC Enable
    | MACCFG2_EXCESSDFR     //  Excess Defer
    ;
  Tr2( "MACCFG2: 0x%X\n", regs->cfg2.reg );

  regs->ipgt.reg = 0x0015; // recommended value for Full Duplex
  Tr2( "MACIPGT: 0x%X\n", regs->ipgt.reg );

  regs->ipgr.reg = 0x0C12; // recommended value
  Tr2( "MACIPGR: 0x%X\n", regs->ipgr.reg );

  //regs->clrt.reg = 0x370F; // default value
  Tr2( "MACCLRT: 0x%X\n", regs->clrt.reg );

  //regs->maxf.reg = 0x05EE; // default value
  Tr2( "MACMAXF: 0x%X\n", regs->maxf.reg );

  Msg( "ETH1: SA %02X:%02X:%02X:%02X:%02X:%02X\n",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5] );

  regs->sa[2].reg = ((uint16_t)(addr[1]) << 8) + addr[0];
  Tr2( "MACSA2: 0x%04X\n", regs->sa[2].reg );
  regs->sa[1].reg = ((uint16_t)(addr[3]) << 8) + addr[2];
  Tr2( "MACSA1: 0x%04X\n", regs->sa[1].reg );
  regs->sa[0].reg = ((uint16_t)(addr[5]) << 8) + addr[4];
  Tr2( "MACSA0: 0x%04X\n", regs->sa[0].reg );
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

// interrupt handler
static void mac_lld_isr(uint32_t _irq, void *data) {
  MACDriver *macp = data;
  ethRegs *regs = (ethRegs *)macp->base;
  uint32_t irq;

  (void)_irq;

  chSysLockFromIsr();

  eicAckIrq(macp->irq);

  Tr3( "ETHIRQ: 0x%X\n", regs->irq.reg );

  // only process enabled interrupts
  irq = regs->irq.reg;
  irq &= regs->ien.reg;
  Tr3( "irq: 0x%X\n", irq );

  if(irq == 0)
    return;

  if( irq & (ETHINT_TXBUSE | ETHINT_TXDONE | ETHINT_TXABORT) ) {
    //int status = irq & (HAL_PIC32MX_ETHINT_TXBUSE | HAL_PIC32MX_ETHINT_TXABORT) ? -1 : 0;
    // TODO: error handling

    // adjust number of available descriptors
    macp->txAvail += macp->txActive;
    macp->txActive = 0;

    // stop transmitter
    pic32eth_tx_stop(macp);

    regs->irq.clear = ETHINT_TXBUSE | ETHINT_TXDONE | ETHINT_TXABORT;

    /* Data Transmitted.*/
    chSemResetI(&ETHD1.tdsem, 0);
  }

  if( irq & (ETHINT_RXBUSE | ETHINT_RXDONE | ETHINT_RXBUFNA | ETHINT_RXOVFLW) )
  {
    regs->irq.clear = ETHINT_RXBUSE | ETHINT_RXDONE | ETHINT_RXBUFNA | ETHINT_RXOVFLW;

    /* Data Received.*/
    chSemResetI(&ETHD1.rdsem, 0);
#if MAC_USE_EVENTS
    chEvtBroadcastI(&ETHD1.rdevent);
#endif
  }

  chSysUnlockFromIsr();
}


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level MAC initialization.
 *
 * @notapi
 */
void mac_lld_init(void) {

  MACDriver *macp;

  Tr1("\n");

#if PLATFORM_MAC_USE_MAC1
  /* Driver initialization.*/
  macObjectInit(&ETHD1);

  macp = &ETHD1;

  // TODO? detect if ETH1 hardware is present

  macp->phy = 1;    // default PHY address, //TODO: make configurable (board-dependent?)

  macp->mii = PIC32MX_DEVCFG_GET(3) & DEVCFG3_FMIIEN ? 1 : 0;
  macp->io = PIC32MX_DEVCFG_GET(3) & DEVCFG3_FETHIO ? 1 : 0;

  Msg( "ETH1: %s;  %s IO\n",
    macp->mii ? "MII" : "RMII", macp->io ? "Default" : "Alternate" );

  // init controller
  pic32eth_ctl_init(macp);
  pic32eth_mac_init(macp);

  // init descriptor lists
  pic32eth_desc_init(macp);

#endif /* PLATFORM_MAC_USE_MAC1 */
}

/**
 * @brief   Configures and activates the MAC peripheral.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
void mac_lld_start(MACDriver *macp) {

  Tr1("\n");

  if (macp->state == MAC_STOP) {
    /* Enables the peripheral.*/
#if PLATFORM_MAC_USE_MAC1
    if (&ETHD1 == macp) {

      ethRegs *regs = (ethRegs *)macp->base;
      uint8_t* addr = macp->config->mac_address;

      // copy pre-programmed ESA into configuration
      //  TODO: config switch to override pre-programmed ESA
      Tr2( "MACSA2: 0x%04X\n", regs->sa[2].reg );
      Tr2( "MACSA1: 0x%04X\n", regs->sa[1].reg );
      Tr2( "MACSA0: 0x%04X\n", regs->sa[0].reg );
      addr[0] = regs->sa[2].reg & 0xFF;
      addr[1] = (regs->sa[2].reg >> 8) & 0xFF;
      addr[2] = regs->sa[1].reg & 0xFF;
      addr[3] = (regs->sa[1].reg >> 8) & 0xFF;
      addr[4] = regs->sa[0].reg & 0xFF;
      addr[5] = (regs->sa[0].reg >> 8) & 0xFF;

      Tr1( "ETH1: SA %02X:%02X:%02X:%02X:%02X:%02X\n",
        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5] );

      // register interrupt handler
      eicRegisterIrq(macp->irq, mac_lld_isr, macp);

      pic32eth_mac_start(macp);
      pic32eth_ctl_start(macp);
    }
#endif /* PLATFORM_MAC_USE_MAC1 */
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Deactivates the MAC peripheral.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
void mac_lld_stop(MACDriver *macp) {

  Tr1( "\n");

  if (macp->state == MAC_ACTIVE) {
    /* Resets the peripheral.*/

    /* Disables the peripheral.*/
#if PLATFORM_MAC_USE_MAC1
    if (&ETHD1 == macp) {
        pic32eth_ctl_init(macp);
        pic32eth_mac_init(macp);

    }
#endif /* PLATFORM_MAC_USE_MAC1 */
  }
}

/**
 * @brief   Returns a transmission descriptor.
 * @details One of the available transmission descriptors is locked and
 *          returned.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] tdp      pointer to a @p MACTransmitDescriptor structure
 * @return              The operation status.
 * @retval RDY_OK       the descriptor has been obtained.
 * @retval RDY_TIMEOUT  descriptor not available.
 *
 * @notapi
 */
msg_t mac_lld_get_transmit_descriptor(MACDriver *macp,
                                      MACTransmitDescriptor *tdp) {
  Tr2( "\n");

  if (!macp->link)
    return RDY_TIMEOUT;

  chSysLock();

  // for now, one active frame at a time
  if (macp->txActive) {
    chSysUnlock();
    return RDY_TIMEOUT;
  }

  // we're currently filling up a descriptor chain
  // in order to keep it contiguous, we don't allow
  // other threads to get a descriptor until current one finishes
  if (macp->txLock) {
    chSysUnlock();
    return RDY_TIMEOUT;
  }

  // as there is no indication of the real frame size the top level
  // driver is about to transmit, we have to wait until there are enough
  // descriptors to transmit MAC_FRAME_SIZE bytes (PIC32_MAC_BUF_PER_FRAME)
  if (macp->txAvail < PIC32_MAC_BUF_PER_FRAME) {
    chSysUnlock();
    return RDY_TIMEOUT;
  }

  // allocate all DMA descriptors necessary to transmit one frame
  // we'll release unused ones later in mac_lld_release_transmit_descriptor
  tdp->macp = macp;
  tdp->offset = 0;
  tdp->size = MAC_BUFFER_SIZE * PIC32_MAC_BUF_PER_FRAME;
  tdp->dma_start = macp->txNext;
  tdp->dma_count = PIC32_MAC_BUF_PER_FRAME;

  macp->txNext += PIC32_MAC_BUF_PER_FRAME;
  if (macp->txNext >= PIC32_TX_DMA_DESCRIPTORS)
    macp->txNext -= PIC32_TX_DMA_DESCRIPTORS;
  macp->txAvail -= PIC32_MAC_BUF_PER_FRAME;
  Tr2( "Next %d  Avail %d\n", macp->txNext, macp->txAvail);

  // save tdp in txLock to prevent other thread(s) from entering
  //  mac_lld_get_transmit_descriptor
  macp->txLock = tdp;

  chSysUnlock();

  Tr2( "start %d  count %d\n", tdp->dma_start, tdp->dma_count);

  return RDY_OK;
}

/**
 * @brief   Releases a transmit descriptor and starts the transmission of the
 *          enqueued data as a single frame.
 *
 * @param[in] tdp       the pointer to the @p MACTransmitDescriptor structure
 *
 * @notapi
 */
void mac_lld_release_transmit_descriptor(MACTransmitDescriptor *tdp) {

  MACDriver *macp = tdp->macp;
  ethRegs *regs = (ethRegs *)macp->base;
  pic32eth_desc* d;
  uint32_t count;
  uint32_t last;
  uint32_t ind;
  uint32_t i;

  chDbgAssert( macp->txLock == tdp,
      "mac_lld_release_transmit_descriptor(), #1",
      "attempt to release invalid descriptor");

  chSysLock();

  // calculate amount of data in the last descriptor
  // and total number of descriptors required
  // to transmit this packet
  last = tdp->offset % MAC_BUFFER_SIZE;
  count = tdp->offset / MAC_BUFFER_SIZE + (last ? 1 : 0);

  Tr1("offset %d  count %d  last %d  start %d\n", tdp->offset, count, last, tdp->dma_start);

  // fill up the descriptors
  ind = tdp->dma_start;
  for (i=0; i<count; i++, ind++ ) {

    if (ind >= PIC32_TX_DMA_DESCRIPTORS)
      ind -= PIC32_TX_DMA_DESCRIPTORS;

    d = &pic32eth_tx.desc[ind];

    if( i == 0 )
      d->sop = 1;
    else
      d->sop = 0;

    if( i == count-1 ) {
      d->eop = 1;
      d->count = last;
    } else {
      d->eop = 0;
      d->count = MAC_BUFFER_SIZE;
    }
  }

  // pass descriptor ownership to the controller
  while (ind != tdp->dma_start) {
    if (ind > 0)
      ind--;
    else
      ind = PIC32_TX_DMA_DESCRIPTORS-1;

    d = &pic32eth_tx.desc[ind];

    d->eown = 1;
  }

  macp->txActive = count;

  // release unused DMA descriptors
  count = tdp->dma_count - count;  // number of descriptors to release
  if (macp->txNext < count)
    macp->txNext = macp->txNext + PIC32_TX_DMA_DESCRIPTORS - count;
  else
    macp->txNext -= count;
  macp->txAvail += count;
  Tr2( "Next %d  Avail %d\n", macp->txNext, macp->txAvail);

  // unlock the tx ring
  macp->txLock = NULL;

  // set transmit start address
  regs->txst.reg = MIPS_PHYSICAL((uint32_t)&pic32eth_tx.desc[tdp->dma_start]);
  Tr3( "ETHTXST: 0x%X\n", regs->txst.reg );

  // start transmitter  // TODO: restart only if inactive
  pic32eth_tx_start(macp);

  chSysUnlock();
}

/**
 * @brief   Returns a receive descriptor.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] rdp      pointer to a @p MACReceiveDescriptor structure
 * @return              The operation status.
 * @retval RDY_OK       the descriptor has been obtained.
 * @retval RDY_TIMEOUT  descriptor not available.
 *
 * @notapi
 */
msg_t mac_lld_get_receive_descriptor(MACDriver *macp,
                                     MACReceiveDescriptor *rdp) {
  pic32eth_desc* d;
  uint32_t i;

  Tr2( "\n");

  chSysLock();

  // a thread is currently reading the descriptor chain
  // we don't allow other threads to get a descriptor until
  // current one finishes
  if (macp->rxLock) {
    chSysUnlock();
    return RDY_TIMEOUT;
  }

  // get first descriptor from rx ring
  d = &pic32eth_rx.desc[macp->rxStart];

  // if descriptor is still owned by eth, stop
  if( d->eown ) {
    chSysUnlock();
    return RDY_TIMEOUT;
  }

  // it MUST be the frame start here
  chDbgAssert( d->sop,
      "mac_lld_get_receive_descriptor, #1",
      "missing SOP");

  rdp->macp = macp;
  rdp->offset = 0;

  // get received frame size minus FCS
  rdp->size = d->rx_count - 4;

  // TODO: check RSV (Status Vector) for errors etc.

  Tr1( "Received frame: %d bytes\n", rdp->size );

  rdp->dma_start = macp->rxStart;
  rdp->dma_count = 1;

  // search for EOP
  i = macp->rxStart;
  while (!d->eop) {

    i++;
    if (i >= PIC32_RX_DMA_DESCRIPTORS)
      i -= PIC32_RX_DMA_DESCRIPTORS;

    d = &pic32eth_rx.desc[i];

    chDbgAssert( (!d->eown) && (i != macp->rxStart),
        "mac_lld_get_receive_descriptor, #1",
        "missing EOP");

    rdp->dma_count++;
  }

  Tr2( "start %d  count %d\n", macp->rxStart, rdp->dma_count);

  macp->rxLock = rdp;

  chSysUnlock();

  return RDY_OK;
}

/**
 * @brief   Releases a receive descriptor.
 * @details The descriptor and its buffer are made available for more incoming
 *          frames.
 *
 * @param[in] rdp       the pointer to the @p MACReceiveDescriptor structure
 *
 * @notapi
 */
void mac_lld_release_receive_descriptor(MACReceiveDescriptor *rdp) {

  MACDriver *macp = rdp->macp;
  ethRegs *regs = (ethRegs *)macp->base;
  uint32_t i;

  Tr2("offset %d\n", rdp->offset);

  chDbgAssert( macp->rxLock == rdp,
      "mac_lld_release_receive_descriptor(), #1",
      "attempt to release invalid descriptor");

  chSysLock();

  // re-init receive descriptors and move ring pointers

  for (i = 0; i < rdp->dma_count; i++) {
    pic32eth_desc* d = &pic32eth_rx.desc[macp->rxStart];

    // re-init the current descriptor, do not set eown yet
    //  it'll be set on next loop while moving ring end
    d->sop = 0;
    d->eop = 0;
    d->Off8 = d->OffC = 0;

    // move ring end
    d = &pic32eth_rx.desc[macp->rxEnd];
    d->eown = 1;
    macp->rxEnd = macp->rxStart;

    // tell controller
    regs->con1.set = ETHCON1_BUFCDEC;

    // move to next descriptor
    macp->rxStart++;
    if (macp->rxStart >= PIC32_RX_DMA_DESCRIPTORS)
      macp->rxStart = 0;
  }

  Tr2( "Start %d  End %d\n", macp->rxStart, macp->rxEnd );

  // unlock the rx ring
  macp->rxLock = NULL;

  chSysUnlock();
}

/**
 * @brief   Updates and returns the link status.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @return              The link status.
 * @retval TRUE         if the link is active.
 * @retval FALSE        if the link is down.
 *
 * @notapi
 */
bool_t mac_lld_poll_link_status(MACDriver *macp) {
  uint16_t bmsr; // PHY Basic Mode Status Register

  bmsr = pic32eth_miim_read(macp, macp->phy, 1);

  macp->link = bmsr & 0x4 ? 1 : 0;

  Tr2( "BMSR 0x%X  link is %s\n", bmsr, macp->link ? "up" : "down");

  return macp->link ? TRUE : FALSE;
}

/**
 * @brief   Writes to a transmit descriptor's stream.
 *
 * @param[in] tdp       pointer to a @p MACTransmitDescriptor structure
 * @param[in] buf       pointer to the buffer containing the data to be
 *                      written
 * @param[in] size      number of bytes to be written
 * @return              The number of bytes written into the descriptor's
 *                      stream, this value can be less than the amount
 *                      specified in the parameter @p size if the maximum
 *                      frame size is reached.
 *
 * @notapi
 */
size_t mac_lld_write_transmit_descriptor(MACTransmitDescriptor *tdp,
                                         uint8_t *buf,
                                         size_t size) {
  size_t sz = size;
  uint32_t ind = tdp->offset / MAC_BUFFER_SIZE;
  uint32_t off = tdp->offset % MAC_BUFFER_SIZE;

  Tr2( "offset %d  size %d\n", tdp->offset, size);

  ind += tdp->dma_start;
  while (sz > 0) {
    pic32eth_desc* d;
    size_t l;

    if (ind >= PIC32_TX_DMA_DESCRIPTORS)
      ind -= PIC32_TX_DMA_DESCRIPTORS;

    d = &pic32eth_tx.desc[ind];
    l = MAC_BUFFER_SIZE - off;

    if (l > sz)
      l = sz;

    chDbgAssert( tdp->offset + l <= MAC_FRAME_SIZE,
        "mac_lld_write_transmit_descriptor(), #1",
        "dma buffer overflow");

    Tr3( "%d: 0x%X+%d <- 0x%X  %d\n", ind, MIPS_UNCACHED(d->addr), off, buf, l );
    memcpy( (uint8_t*)MIPS_UNCACHED(d->addr)+off, buf, l );

    Tr3( "  %x %x %x %x %x %x  %x %x %x %x %x %x  %x %x\n",
        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
        buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
        buf[12], buf[13] );

    ind++;
    off = 0;
    buf += l;
    sz -= l;
    tdp->offset += l;
  }

  return size;
}

/**
 * @brief   Reads from a receive descriptor's stream.
 *
 * @param[in] rdp       pointer to a @p MACReceiveDescriptor structure
 * @param[in] buf       pointer to the buffer that will receive the read data
 * @param[in] size      number of bytes to be read
 * @return              The number of bytes read from the descriptor's
 *                      stream, this value can be less than the amount
 *                      specified in the parameter @p size if there are
 *                      no more bytes to read.
 *
 * @notapi
 */
size_t mac_lld_read_receive_descriptor(MACReceiveDescriptor *rdp,
                                       uint8_t *buf,
                                       size_t size) {

  size_t sz = size;
  uint32_t ind = rdp->offset / MAC_BUFFER_SIZE;
  uint32_t off = rdp->offset % MAC_BUFFER_SIZE;

  Tr2( "offset %d  size %d\n", rdp->offset, size);

  ind += rdp->dma_start;
  while (sz > 0) {
    pic32eth_desc* d;
    size_t l;

    if (ind >= PIC32_RX_DMA_DESCRIPTORS)
      ind -= PIC32_RX_DMA_DESCRIPTORS;

    d = &pic32eth_rx.desc[ind];
    l = d->count - off;

    if (l > sz)
      l = sz;

    chDbgAssert( rdp->offset + l <= rdp->size,
        "mac_lld_read_receive_descriptor(), #1",
        "dma buffer underflow");

    Tr3( "%d: 0x%X <- 0x%X+%d %d\n", ind, buf, MIPS_UNCACHED(d->addr), off, l );
    memcpy( buf, (uint8_t*)MIPS_UNCACHED(d->addr)+off, l );

    Tr3( "  %x %x %x %x %x %x  %x %x %x %x %x %x  %x %x\n",
        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
        buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
        buf[12], buf[13] );

    ind++;
    off = 0;
    buf += l;
    sz -= l;
    rdp->offset += l;
  }

  return size;
}

#if MAC_USE_ZERO_COPY || defined(__DOXYGEN__)
/**
 * @brief   Returns a pointer to the next transmit buffer in the descriptor
 *          chain.
 * @note    The API guarantees that enough buffers can be requested to fill
 *          a whole frame.
 *
 * @param[in] tdp       pointer to a @p MACTransmitDescriptor structure
 * @param[in] size      size of the requested buffer. Specify the frame size
 *                      on the first call then scale the value down subtracting
 *                      the amount of data already copied into the previous
 *                      buffers.
 * @param[out] sizep    pointer to variable receiving the buffer size, it is
 *                      zero when the last buffer has already been returned.
 *                      Note that a returned size lower than the amount
 *                      requested means that more buffers must be requested
 *                      in order to fill the frame data entirely.
 * @return              Pointer to the returned buffer.
 * @retval NULL         if the buffer chain has been entirely scanned.
 *
 * @notapi
 */
uint8_t *mac_lld_get_next_transmit_buffer(MACTransmitDescriptor *tdp,
                                          size_t size,
                                          size_t *sizep) {

  (void)tdp;
  (void)size;
  (void)sizep;

  return NULL;
}

/**
 * @brief   Returns a pointer to the next receive buffer in the descriptor
 *          chain.
 * @note    The API guarantees that the descriptor chain contains a whole
 *          frame.
 *
 * @param[in] rdp       pointer to a @p MACReceiveDescriptor structure
 * @param[out] sizep    pointer to variable receiving the buffer size, it is
 *                      zero when the last buffer has already been returned.
 * @return              Pointer to the returned buffer.
 * @retval NULL         if the buffer chain has been entirely scanned.
 *
 * @notapi
 */
const uint8_t *mac_lld_get_next_receive_buffer(MACReceiveDescriptor *rdp,
                                               size_t *sizep) {

  (void)rdp;
  (void)sizep;

  return NULL;
}
#endif /* MAC_USE_ZERO_COPY */

#endif /* HAL_USE_MAC */

/** @} */
